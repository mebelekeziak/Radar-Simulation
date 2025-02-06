using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RealRadarSim.Logging;
using RealRadarSim.Models;

namespace RealRadarSim.Tracking
{
    /// <summary>
    /// Manages tracks and performs the Joint Probabilistic Data Association (JPDA)
    /// update using adaptive gating, IMM filtering (EKF+UKF), candidate track initiation,
    /// merging, and pruning.
    /// </summary>
    public class TrackManager
    {
        private List<JPDA_Track> tracks = new List<JPDA_Track>();
        private List<CandidateTrack> candidates = new List<CandidateTrack>();
        private int nextTrackId = 1;
        private Random rng;

        // --- Tuning and model parameters ---
        public double InitGateThreshold { get; set; } = 14.07;
        public int InitRequiredHits { get; set; } = 3;
        public int InitScanWindow { get; set; } = 3;
        public double InitPosStd { get; set; } = 200.0;
        public double InitVelStd { get; set; } = 100.0;
        public double GatingThreshold { get; set; } = 35.0;   // Base chi-square threshold
        public double AccelNoise { get; set; } = 2.0;
        public double ProbDetection { get; set; } = 0.9;
        public double ProbSurvival { get; set; } = 0.995;
        public double PruneThreshold { get; set; } = 0.01;
        public double MaxTrackMergeDist { get; set; } = 800.0;
        public double MaxTrackAge { get; set; } = 40.0;
        public double CandidateMergeDistance { get; set; } = 1500.0;
        public double ClutterDensity { get; set; } = 1e-6;
        public double ExistenceIncreaseGain { get; set; } = 0.05;
        public double ExistenceDecayFactor { get; set; } = 0.995;

        public TrackManager(Random rng)
        {
            this.rng = rng;
        }

        /// <summary>
        /// Main update loop to predict, associate, update, initiate, merge, and prune tracks.
        /// </summary>
        /// <param name="measurements">List of radar measurements (range, azimuth, elevation, amplitude).</param>
        /// <param name="dt">Time step since the last update (seconds).</param>
        public void UpdateTracks(List<Measurement> measurements, double dt)
        {
            // 1) Prediction Step: advance each track’s filter.
            foreach (var track in tracks)
            {
                track.Filter.Predict(dt);
                track.Age += dt;
            }

            // 2) Gating: for each track, decide which measurements fall within its (adaptive) gate.
            List<List<int>> gateMatrix = new List<List<int>>();
            for (int i = 0; i < tracks.Count; i++)
                gateMatrix.Add(new List<int>());

            for (int i = 0; i < tracks.Count; i++)
            {
                var track = tracks[i];
                double speed = track.Filter.State.SubVector(3, 3).L2Norm();
                double dynamicFactor = 1.0 + 0.5 * (speed / 100.0);
                double existenceFactor = (track.ExistenceProb > 0.7) ? 1.5 : 1.0;
                double adaptiveThreshold = GatingThreshold * dynamicFactor * existenceFactor;
                adaptiveThreshold = Math.Max(5.0, Math.Min(adaptiveThreshold, 100.0)); // clamp

                for (int m = 0; m < measurements.Count; m++)
                {
                    double md2 = MahalanobisDistanceSq(track, measurements[m]);
                    if (md2 < adaptiveThreshold)
                        gateMatrix[i].Add(m);
                }
            }

            // 3) Build measurement-to-track associations.
            Dictionary<int, List<int>> measToTracks = new Dictionary<int, List<int>>();
            for (int m = 0; m < measurements.Count; m++)
                measToTracks[m] = new List<int>();

            for (int i = 0; i < tracks.Count; i++)
            {
                foreach (int m in gateMatrix[i])
                    measToTracks[m].Add(i);
            }

            // 4) Compute likelihoods L(i, m) for each track-measurement pair.
            List<Dictionary<int, double>> trackMeasLikelihood = new List<Dictionary<int, double>>();
            for (int i = 0; i < tracks.Count; i++)
                trackMeasLikelihood.Add(new Dictionary<int, double>());

            for (int m = 0; m < measurements.Count; m++)
            {
                var trackList = measToTracks[m];
                if (trackList.Count == 0) continue;
                Vector<double> z = ToVector(measurements[m]);
                foreach (int i in trackList)
                {
                    double likelihood = ComputeMeasurementLikelihood(tracks[i], z);
                    trackMeasLikelihood[i][m] = likelihood;
                }
            }

            // 5) For each measurement, sum likelihoods over all gating tracks.
            double[] measurementLikelihoodSum = new double[measurements.Count];
            for (int m = 0; m < measurements.Count; m++)
                measurementLikelihoodSum[m] = 0.0;
            for (int i = 0; i < tracks.Count; i++)
            {
                foreach (var pair in trackMeasLikelihood[i])
                {
                    int m = pair.Key;
                    measurementLikelihoodSum[m] += ProbDetection * pair.Value;
                }
            }

            // 6) Compute association probabilities β and false-alarm probability β0.
            // Instead of using a constant gating volume, compute a track-specific gating volume based on its predicted measurement covariance.
            double[] beta0 = new double[measurements.Count];
            Dictionary<(int, int), double> beta = new Dictionary<(int, int), double>();
            for (int m = 0; m < measurements.Count; m++)
            {
                // For measurements associated with multiple tracks, average their gating volumes.
                List<double> lambdaV_list = new List<double>();
                foreach (int i in measToTracks[m])
                {
                    Matrix<double> S = tracks[i].Filter.S();
                    double detS = S.Determinant();
                    if (detS < 1e-12) detS = 1e-12;
                    // Use the track’s adaptive threshold (approximated here by GatingThreshold)
                    double gatingVolume_i = (4.0 / 3.0) * Math.PI * Math.Pow(Math.Sqrt(GatingThreshold), 3) * Math.Sqrt(detS);
                    lambdaV_list.Add(ClutterDensity * gatingVolume_i);
                }
                double lambdaV_m = (lambdaV_list.Count > 0) ? lambdaV_list.Average() : ClutterDensity;
                double denominator = measurementLikelihoodSum[m] + lambdaV_m;
                if (denominator < 1e-12) denominator = 1e-12;
                beta0[m] = lambdaV_m / denominator;
                foreach (int i in measToTracks[m])
                {
                    double numerator = ProbDetection * trackMeasLikelihood[i][m];
                    beta[(i, m)] = numerator / denominator;
                }
            }

            // 7) Update each track with a weighted (effective) measurement.
            for (int i = 0; i < tracks.Count; i++)
            {
                double sumBeta = 0.0;
                Vector<double> zEff = DenseVector.Create(3, 0.0);
                // First, compute the weighted measurement mean.
                foreach (int m in gateMatrix[i])
                {
                    double b = beta.ContainsKey((i, m)) ? beta[(i, m)] : 0.0;
                    sumBeta += b;
                    zEff += ToVector(measurements[m]) * b;
                }
                if (sumBeta > 1e-9)
                {
                    zEff = zEff / sumBeta;
                    // Compute the measurement scatter (covariance) from the gated measurements.
                    Matrix<double> Pzz = DenseMatrix.Create(3, 3, 0.0);
                    foreach (int m in gateMatrix[i])
                    {
                        double b = beta.ContainsKey((i, m)) ? beta[(i, m)] : 0.0;
                        if (b < 1e-12) continue;
                        Vector<double> diff = ToVector(measurements[m]) - zEff;
                        diff[1] = NormalizeAngle(diff[1]);
                        diff[2] = NormalizeAngle(diff[2]);
                        Pzz += b * (diff.ToColumnMatrix() * diff.ToRowMatrix());
                    }
                    Pzz /= sumBeta;
                    // Add the intrinsic measurement noise from the filter.
                    Matrix<double> R_meas = tracks[i].Filter.GetMeasurementNoiseCov();
                    Matrix<double> S_jpda = Pzz + R_meas;
                    // Use the new update overload that accepts custom measurement covariance.
                    tracks[i].Filter.Update(zEff, S_jpda);
                    tracks[i].Age = 0;
                    tracks[i].ExistenceProb = Math.Min(1.0, tracks[i].ExistenceProb + ExistenceIncreaseGain * sumBeta);
                }
                else
                {
                    // No valid measurement associated—decay existence.
                    tracks[i].ExistenceProb *= ExistenceDecayFactor;
                    string reason = (gateMatrix[i].Count == 0)
                        ? "No measurements passed the gating threshold."
                        : "Measurements passed gating but association likelihood too low.";
                    DebugLogger.WriteLine(
                        $"[Missed Track] TrackID {tracks[i].TrackId}: Missed update. Reason: {reason} " +
                        $"Existence decayed to {tracks[i].ExistenceProb:F2}");
                }
            }

            // 8) Update candidate tracks from measurements not clearly associated with existing tracks.
            CreateOrUpdateCandidates(measurements, beta0);

            // 9) Confirm candidate tracks (initiate new tracks)
            ConfirmCandidates();

            // 10) Merge tracks that are close together using Gaussian fusion.
            MergeCloseTracks();

            // 11) Prune tracks that are too weak or too old.
            PruneTracks();
        }

        #region Helper Methods

        private double MahalanobisDistanceSq(JPDA_Track track, Measurement m)
        {
            Vector<double> zPred = track.Filter.H(track.Filter.State);
            Vector<double> zMeas = ToVector(m);
            Vector<double> y = zMeas - zPred;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);
            Matrix<double> S = track.Filter.S();
            Matrix<double> SInv = S.Inverse();
            double dist2 = (y.ToRowMatrix() * SInv * y.ToColumnMatrix())[0, 0];
            return dist2;
        }

        private double ComputeMeasurementLikelihood(JPDA_Track track, Vector<double> z)
        {
            Vector<double> zPred = track.Filter.H(track.Filter.State);
            Vector<double> y = z - zPred;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);
            Matrix<double> S = track.Filter.S();
            double det = S.Determinant();
            if (det < 1e-12) det = 1e-12;
            Matrix<double> SInv = S.Inverse();
            double dist2 = (y.ToRowMatrix() * SInv * y.ToColumnMatrix())[0, 0];
            double normFactor = 1.0 / (Math.Pow(2 * Math.PI, 1.5) * Math.Sqrt(det));
            return normFactor * Math.Exp(-0.5 * dist2);
        }

        private Vector<double> ToVector(Measurement m)
        {
            return DenseVector.OfArray(new double[] { m.Range, m.Azimuth, m.Elevation });
        }

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        private (Vector<double> state, Matrix<double> cov) MeasurementToInitialState(Vector<double> meas)
        {
            double r = meas[0];
            double az = meas[1];
            double el = meas[2];
            double x = r * Math.Cos(el) * Math.Cos(az);
            double y = r * Math.Cos(el) * Math.Sin(az);
            double z = r * Math.Sin(el);
            Vector<double> state = DenseVector.OfArray(new double[] { x, y, z, 0, 0, 0 });
            Matrix<double> cov = DenseMatrix.Create(6, 6, 0.0);
            cov[0, 0] = InitPosStd * InitPosStd;
            cov[1, 1] = InitPosStd * InitPosStd;
            cov[2, 2] = InitPosStd * InitPosStd;
            cov[3, 3] = InitVelStd * InitVelStd;
            cov[4, 4] = InitVelStd * InitVelStd;
            cov[5, 5] = InitVelStd * InitVelStd;
            return (state, cov);
        }

        // --- Candidate Track Management ---
        private void CreateOrUpdateCandidates(List<Measurement> measurements, double[] beta0)
        {
            HashSet<int> associated = new HashSet<int>();
            for (int m = 0; m < measurements.Count; m++)
            {
                if (beta0[m] < 0.85)
                    associated.Add(m);
            }
            for (int m = 0; m < measurements.Count; m++)
            {
                if (associated.Contains(m)) continue;
                if (!IsWithinAnyExistingTrack(measurements[m]))
                    CreateOrUpdateCandidate(measurements[m]);
            }
        }

        private bool IsWithinAnyExistingTrack(Measurement m)
        {
            double rescueRadius = 3000.0;
            Vector<double> mPos = MeasurementToCartesian(m);
            foreach (var track in tracks)
            {
                Vector<double> tPos = track.Filter.State.SubVector(0, 3);
                if ((tPos - mPos).L2Norm() < rescueRadius)
                    return true;
            }
            return false;
        }

        private void CreateOrUpdateCandidate(Measurement m)
        {
            double bestDist = double.MaxValue;
            CandidateTrack bestCandidate = null;
            foreach (var cand in candidates)
            {
                Measurement lastMeas = cand.Measurements.Last();
                double d = SphericalDistance(m, lastMeas);
                if (d < CandidateMergeDistance && d < bestDist)
                {
                    bestDist = d;
                    bestCandidate = cand;
                }
            }
            if (bestCandidate != null)
                bestCandidate.Measurements.Add(m);
            else
            {
                CandidateTrack newCandidate = new CandidateTrack();
                newCandidate.Measurements.Add(m);
                candidates.Add(newCandidate);
            }
        }

        private double SphericalDistance(Measurement a, Measurement b)
        {
            double xA = a.Range * Math.Cos(a.Elevation) * Math.Cos(a.Azimuth);
            double yA = a.Range * Math.Cos(a.Elevation) * Math.Sin(a.Azimuth);
            double zA = a.Range * Math.Sin(a.Elevation);
            double xB = b.Range * Math.Cos(b.Elevation) * Math.Cos(b.Azimuth);
            double yB = b.Range * Math.Cos(b.Elevation) * Math.Sin(b.Azimuth);
            double zB = b.Range * Math.Sin(b.Elevation);
            double dx = xA - xB;
            double dy = yA - yB;
            double dz = zA - zB;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private void ConfirmCandidates()
        {
            List<CandidateTrack> toConfirm = new List<CandidateTrack>();
            foreach (var cand in candidates)
            {
                if (cand.Measurements.Count >= InitRequiredHits)
                    toConfirm.Add(cand);
            }
            foreach (var cand in toConfirm)
            {
                Vector<double> zSum = DenseVector.Create(3, 0.0);
                foreach (var meas in cand.Measurements)
                    zSum += ToVector(meas);
                Vector<double> zAvg = zSum / cand.Measurements.Count;
                (Vector<double> initState, Matrix<double> initCov) = MeasurementToInitialState(zAvg);

                ITrackerFilter ekf = new EKFFilter(initState, initCov, AccelNoise);
                ITrackerFilter ukf = new UKFFilter(initState, initCov, AccelNoise);
                ITrackerFilter imm = new IMMFilter(ekf, ukf);

                JPDA_Track newTrack = new JPDA_Track
                {
                    TrackId = nextTrackId++,
                    Filter = imm,
                    Age = 0,
                    ExistenceProb = 0.5
                };
                tracks.Add(newTrack);
                candidates.Remove(cand);
            }
        }

        // --- Track Merging and Pruning ---
        private void MergeCloseTracks()
        {
            double posMergeThreshold = 7.0;    // Mahalanobis threshold for position
            double velMergeThreshold = 15.0;   // max velocity difference in m/s
            List<JPDA_Track> toRemove = new List<JPDA_Track>();

            for (int i = 0; i < tracks.Count; i++)
            {
                for (int j = i + 1; j < tracks.Count; j++)
                {
                    JPDA_Track t1 = tracks[i];
                    JPDA_Track t2 = tracks[j];

                    double posMd = TrackMahalanobisDistance(t1, t2);
                    Vector<double> velDiff = t1.Filter.State.SubVector(3, 3) - t2.Filter.State.SubVector(3, 3);
                    double velDiffNorm = velDiff.L2Norm();

                    if (posMd < posMergeThreshold && velDiffNorm < velMergeThreshold)
                    {
                        // Gaussian fusion: x_merged = (P1^-1+P2^-1)^-1(P1^-1x1+P2^-1x2)
                        Matrix<double> P1Inv = t1.Filter.Covariance.Inverse();
                        Matrix<double> P2Inv = t2.Filter.Covariance.Inverse();
                        Matrix<double> fusedCov = (P1Inv + P2Inv).Inverse();
                        Vector<double> fusedState = fusedCov * (P1Inv * t1.Filter.State + P2Inv * t2.Filter.State);
                        t2.Filter.State = fusedState;
                        t2.Filter.Covariance = fusedCov;
                        t2.ExistenceProb = Math.Max(t2.ExistenceProb, t1.ExistenceProb);
                        toRemove.Add(t1);
                    }
                }
            }
            tracks.RemoveAll(t => toRemove.Contains(t));
        }

        private double TrackMahalanobisDistance(JPDA_Track t1, JPDA_Track t2)
        {
            Vector<double> diff = t1.Filter.State.SubVector(0, 3) - t2.Filter.State.SubVector(0, 3);
            Matrix<double> cov1 = t1.Filter.Covariance.SubMatrix(0, 3, 0, 3);
            Matrix<double> cov2 = t2.Filter.Covariance.SubMatrix(0, 3, 0, 3);
            Matrix<double> covSum = cov1 + cov2;
            Matrix<double> invCov = covSum.Inverse();
            double md2 = (diff.ToRowMatrix() * invCov * diff.ToColumnMatrix())[0, 0];
            return Math.Sqrt(md2);
        }

        private void PruneTracks()
        {
            tracks.RemoveAll(t => (t.ExistenceProb < PruneThreshold) || (t.Age > MaxTrackAge));
        }

        /// <summary>
        /// Returns the list of confirmed tracks.
        /// </summary>
        public List<JPDA_Track> GetTracks() => tracks;

        private Vector<double> MeasurementToCartesian(Measurement m)
        {
            double x = m.Range * Math.Cos(m.Elevation) * Math.Cos(m.Azimuth);
            double y = m.Range * Math.Cos(m.Elevation) * Math.Sin(m.Azimuth);
            double z = m.Range * Math.Sin(m.Elevation);
            return DenseVector.OfArray(new double[] { x, y, z });
        }

        #endregion
    }

    /// <summary>
    /// A candidate track that accumulates measurements until confirmed.
    /// </summary>
    public class CandidateTrack
    {
        public List<Measurement> Measurements = new List<Measurement>();
    }

    /// <summary>
    /// A confirmed track.
    /// </summary>
    public class JPDA_Track
    {
        public int TrackId;
        public ITrackerFilter Filter;
        public double Age;
        public double ExistenceProb;
        public string FlightName; // Optional flight or target name.
    }
}

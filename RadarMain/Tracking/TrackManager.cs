using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RealRadarSim.Models;

namespace RealRadarSim.Tracking
{
    /// <summary>
    /// Manages tracks and performs the Joint Probabilistic Data Association (JPDA)
    /// update using adaptive gating, candidate track initiation, merging, and pruning.
    /// </summary>
    public class TrackManager
    {
        private List<JPDA_Track> tracks = new List<JPDA_Track>();
        private List<CandidateTrack> candidates = new List<CandidateTrack>();
        private int nextTrackId = 1;
        private Random rng;

        // --- Tuning and model parameters ---
        // Parameters for candidate initiation and gating
        public double InitGateThreshold { get; set; } = 14.07;
        public int InitRequiredHits { get; set; } = 3;
        public int InitScanWindow { get; set; } = 3;
        public double InitPosStd { get; set; } = 10.0;
        public double InitVelStd { get; set; } = 5.0;
        // NEW: Standard deviation for initial acceleration uncertainty.
        public double InitAccelStd { get; set; } = 2.0;

        // Base probability for the chi-square gate (set to 0.99 for 99% coverage)
        public double GatingProbability { get; set; } = 0.997;
        public double AccelNoise { get; set; } = 2.0;
        public double ProbDetection { get; set; } = 0.9;
        public double ProbSurvival { get; set; } = 0.995;
        public double PruneThreshold { get; set; } = 0.01;
        public double MaxTrackMergeDist { get; set; } = 800.0;
        public double MaxTrackAge { get; set; } = 40.0;
        public double CandidateMergeDistance { get; set; } = 1500.0;
        public double ClutterDensity { get; set; } = 1e-6;
        public double ExistenceIncreaseGain { get; set; } = 0.2;
        public double ExistenceDecayFactor { get; set; } = 0.995;

        // Measurement dimension (range, azimuth, elevation)
        private int measurementDimension = 3;

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

            // 2) Gating using a chi-square threshold computed from the gating probability.
            // The threshold is further scaled by dynamic and existence factors.
            List<List<int>> gateMatrix = new List<List<int>>();
            for (int i = 0; i < tracks.Count; i++)
                gateMatrix.Add(new List<int>());

            // Create a chi-squared distribution for the measurement dimension.
            var chiSqDist = new ChiSquared(measurementDimension);
            double baseGateThreshold = chiSqDist.InverseCumulativeDistribution(GatingProbability);

            for (int i = 0; i < tracks.Count; i++)
            {
                var track = tracks[i];
                // Adjust the threshold based on track dynamics.
                double speed = track.Filter.State.SubVector(3, 3).L2Norm();
                double dynamicFactor = 1.0 + 0.5 * (speed / 100.0);
                double existenceFactor = (track.ExistenceProb > 0.7) ? 1.5 : 1.0;
                double adaptiveThreshold = baseGateThreshold * dynamicFactor * existenceFactor;
                // Clamp to a reasonable range.
                adaptiveThreshold = Math.Max(5.0, Math.Min(adaptiveThreshold, 1e6));

                // For each measurement, compute the Mahalanobis distance squared.
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

            // 6) Compute association probabilities β and false–alarm probability β₀.
            double[] beta0 = new double[measurements.Count];
            Dictionary<(int, int), double> beta = new Dictionary<(int, int), double>();

            for (int m = 0; m < measurements.Count; m++)
            {
                // Compute a base gating volume from the chi-square threshold.
                // Using a spherical approximation:
                double baseGatingVolume = (4.0 / 3.0) * Math.PI * Math.Pow(Math.Sqrt(baseGateThreshold), 3);

                // If one or more tracks gated on this measurement, adjust the clutter likelihood
                // using the average square root of the innovation covariance determinant.
                double avgSqrtDetS = 1.0;  // Default value if no tracks gate on the measurement.
                if (measToTracks[m].Count > 0)
                {
                    double sumSqrtDetS = 0.0;
                    foreach (int i in measToTracks[m])
                    {
                        Matrix<double> S = tracks[i].Filter.S();
                        double detS = Math.Max(S.Determinant(), 1e-12);
                        sumSqrtDetS += Math.Sqrt(detS);
                    }
                    avgSqrtDetS = sumSqrtDetS / measToTracks[m].Count;
                }

                // Compute the effective clutter likelihood for measurement m.
                double clutterLikelihood = ClutterDensity * baseGatingVolume * avgSqrtDetS;

                // Total likelihood: sum of track likelihoods plus the clutter likelihood.
                double totalLikelihood = measurementLikelihoodSum[m] + clutterLikelihood;
                if (totalLikelihood < 1e-12)
                    totalLikelihood = 1e-12;

                // β₀ is the fraction of the likelihood attributed to clutter.
                beta0[m] = clutterLikelihood / totalLikelihood;

                // For each track gating measurement m, compute β(i, m).
                foreach (int i in measToTracks[m])
                {
                    double associationProb = (ProbDetection * trackMeasLikelihood[i][m]) / totalLikelihood;
                    beta[(i, m)] = associationProb;
                }
            }

            // 7) Update each track with a weighted (effective) measurement.
            for (int i = 0; i < tracks.Count; i++)
            {
                double sumBeta = 0.0;
                Vector<double> zEff = DenseVector.Create(3, 0.0);
                // Compute the weighted measurement mean.
                foreach (int m in gateMatrix[i])
                {
                    double b = beta.ContainsKey((i, m)) ? beta[(i, m)] : 0.0;
                    sumBeta += b;
                    zEff += ToVector(measurements[m]) * b;
                }
                if (sumBeta > 1e-9)
                {
                    zEff /= sumBeta;
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
                    // Incorporate the intrinsic measurement noise from the filter.
                    Matrix<double> R_meas = tracks[i].Filter.GetMeasurementNoiseCov();
                    Matrix<double> S_jpda = Pzz + R_meas;
                    tracks[i].Filter.Update(zEff, S_jpda);
                    tracks[i].Age = 0;
                    tracks[i].ExistenceProb = Math.Min(1.0, tracks[i].ExistenceProb + ExistenceIncreaseGain * sumBeta);
                }
                else
                {
                    // No valid measurement associated—decay the existence probability.
                    tracks[i].ExistenceProb *= ExistenceDecayFactor;
                }
            }

            // 8) Update candidate tracks from measurements not clearly associated with existing tracks.
            CreateOrUpdateCandidates(measurements, beta0);

            // 9) Confirm candidate tracks (initiate new tracks).
            ConfirmCandidates();

            // 10) Merge tracks that are close together using DBSCAN-based Gaussian fusion.
            MergeTracksDBSCAN();

            // 11) Prune tracks that are too weak or too old.
            PruneTracks();
        }

        #region DBSCAN-Based Track Merging Methods

        /// <summary>
        /// Clusters tracks using DBSCAN and fuses each cluster’s tracks via Gaussian fusion.
        /// </summary>
        private void MergeTracksDBSCAN()
        {
            int n = tracks.Count;
            if (n == 0)
                return;

            // Initialize cluster IDs (-1 means unassigned) and visited flag for each track.
            int[] clusterIds = new int[n];
            for (int i = 0; i < n; i++)
                clusterIds[i] = -1;
            bool[] visited = new bool[n];

            // Extract each track’s position (first three state elements).
            var positions = new (double x, double y, double z)[n];
            for (int i = 0; i < n; i++)
            {
                var pos = tracks[i].Filter.State.SubVector(0, 3);
                positions[i] = (pos[0], pos[1], pos[2]);
            }

            // DBSCAN parameters.
            int clusterId = 0;
            int minPts = 1;       // Even a single track can form its own cluster.
            double eps = 50.0;    // Merge threshold in meters (adjustable as needed).

            // Perform DBSCAN clustering.
            for (int i = 0; i < n; i++)
            {
                if (!visited[i])
                {
                    visited[i] = true;
                    List<int> neighborIndices = RegionQueryPositions(positions, i, eps);
                    if (neighborIndices.Count < minPts)
                    {
                        // Mark as noise (we treat noise as its own cluster, id 0).
                        clusterIds[i] = 0;
                    }
                    else
                    {
                        clusterId++;
                        ExpandClusterPositions(positions, i, neighborIndices, clusterId, eps, minPts, visited, clusterIds);
                    }
                }
            }

            // Group tracks by their cluster ID.
            Dictionary<int, List<JPDA_Track>> clusters = new Dictionary<int, List<JPDA_Track>>();
            for (int i = 0; i < n; i++)
            {
                int cid = clusterIds[i];
                if (!clusters.ContainsKey(cid))
                    clusters[cid] = new List<JPDA_Track>();
                clusters[cid].Add(tracks[i]);
            }

            // For each cluster, fuse the tracks using Gaussian fusion.
            List<JPDA_Track> mergedTracks = new List<JPDA_Track>();
            foreach (var kvp in clusters)
            {
                var clusterTracks = kvp.Value;
                if (clusterTracks.Count == 1)
                {
                    mergedTracks.Add(clusterTracks[0]);
                }
                else
                {
                    // Fuse all tracks in the cluster iteratively.
                    JPDA_Track mergedTrack = clusterTracks[0];
                    for (int i = 1; i < clusterTracks.Count; i++)
                    {
                        mergedTrack = FuseTracks(mergedTrack, clusterTracks[i]);
                    }
                    mergedTracks.Add(mergedTrack);
                }
            }

            // Replace the current track list with the merged tracks.
            tracks = mergedTracks;
        }

        /// <summary>
        /// Returns a list of indices for tracks within 'eps' distance.
        /// </summary>
        private List<int> RegionQueryPositions((double x, double y, double z)[] positions, int index, double eps)
        {
            List<int> neighbors = new List<int>();
            var point = positions[index];
            for (int i = 0; i < positions.Length; i++)
            {
                if (Distance(point, positions[i]) <= eps)
                    neighbors.Add(i);
            }
            return neighbors;
        }

        /// <summary>
        /// Computes the Euclidean distance between two 3D points.
        /// </summary>
        private double Distance((double x, double y, double z) a, (double x, double y, double z) b)
        {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            double dz = a.z - b.z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Expands the DBSCAN cluster for the given track index.
        /// </summary>
        private void ExpandClusterPositions(
            (double x, double y, double z)[] positions,
            int index,
            List<int> neighborIndices,
            int clusterId,
            double eps,
            int minPts,
            bool[] visited,
            int[] clusterIds)
        {
            clusterIds[index] = clusterId;
            Queue<int> seeds = new Queue<int>(neighborIndices);
            while (seeds.Count > 0)
            {
                int current = seeds.Dequeue();
                if (!visited[current])
                {
                    visited[current] = true;
                    List<int> currentNeighbors = RegionQueryPositions(positions, current, eps);
                    if (currentNeighbors.Count >= minPts)
                    {
                        foreach (int n in currentNeighbors)
                        {
                            if (!seeds.Contains(n))
                                seeds.Enqueue(n);
                        }
                    }
                }
                if (clusterIds[current] == -1)
                    clusterIds[current] = clusterId;
            }
        }

        /// <summary>
        /// Fuses two tracks using Gaussian fusion.
        /// </summary>
        private JPDA_Track FuseTracks(JPDA_Track t1, JPDA_Track t2)
        {
            Matrix<double> P1Inv = t1.Filter.Covariance.Inverse();
            Matrix<double> P2Inv = t2.Filter.Covariance.Inverse();
            Matrix<double> fusedCov = (P1Inv + P2Inv).Inverse();
            Vector<double> fusedState = fusedCov * (P1Inv * t1.Filter.State + P2Inv * t2.Filter.State);

            // Update one track with the fused state and covariance.
            t2.Filter.State = fusedState;
            t2.Filter.Covariance = fusedCov;
            t2.ExistenceProb = Math.Max(t2.ExistenceProb, t1.ExistenceProb);
            return t2;
        }

        #endregion

        #region Helper Methods

        /// <summary>
        /// Computes the Mahalanobis distance squared between the predicted measurement and the actual measurement.
        /// </summary>
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

        /// <summary>
        /// Computes the measurement likelihood for a given track and measurement.
        /// </summary>
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
            double normFactor = 1.0 / (Math.Pow(2 * Math.PI, measurementDimension / 2.0) * Math.Sqrt(det));
            double likelihood = normFactor * Math.Exp(-0.5 * dist2);
            return likelihood;
        }

        /// <summary>
        /// Converts a Measurement to a 3D vector (range, azimuth, elevation).
        /// </summary>
        private Vector<double> ToVector(Measurement m)
        {
            return DenseVector.OfArray(new double[] { m.Range, m.Azimuth, m.Elevation });
        }

        /// <summary>
        /// Normalizes an angle to the range [-π, π].
        /// </summary>
        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        /// <summary>
        /// Converts a measurement in spherical coordinates to an initial Cartesian state.
        /// 
        /// UPDATED for a 9-dimensional state:
        /// [px, py, pz, vx, vy, vz, ax, ay, az]
        /// </summary>
        private (Vector<double> state, Matrix<double> cov) MeasurementToInitialState(Vector<double> meas)
        {
            double r = meas[0];
            double az = meas[1];
            double el = meas[2];
            double x = r * Math.Cos(el) * Math.Cos(az);
            double y = r * Math.Cos(el) * Math.Sin(az);
            double z = r * Math.Sin(el);
            // Create a 9-dimensional state: positions; zero velocity; zero acceleration.
            Vector<double> state = DenseVector.OfArray(new double[] { x, y, z, 0, 0, 0, 0, 0, 0 });
            // Create a 9x9 covariance matrix.
            Matrix<double> cov = DenseMatrix.Create(9, 9, 0.0);
            // Position uncertainties.
            cov[0, 0] = InitPosStd * InitPosStd;
            cov[1, 1] = InitPosStd * InitPosStd;
            cov[2, 2] = InitPosStd * InitPosStd;
            // Velocity uncertainties.
            cov[3, 3] = InitVelStd * InitVelStd;
            cov[4, 4] = InitVelStd * InitVelStd;
            cov[5, 5] = InitVelStd * InitVelStd;
            // Acceleration uncertainties.
            cov[6, 6] = InitAccelStd * InitAccelStd;
            cov[7, 7] = InitAccelStd * InitAccelStd;
            cov[8, 8] = InitAccelStd * InitAccelStd;
            return (state, cov);
        }

        /// <summary>
        /// Returns a formatted string that identifies the measurement.
        /// </summary>
        private string GetMeasurementName(Measurement m)
        {
            string targetName = !string.IsNullOrEmpty(m.TargetName) ? m.TargetName : "Unknown";
            return $"[Target: {targetName}, Range: {m.Range:F2}, Az: {m.Azimuth:F2}, El: {m.Elevation:F2}, Amp: {m.Amplitude:F2}]";
        }

        /// <summary>
        /// Converts a measurement in spherical coordinates to a Cartesian vector.
        /// </summary>
        private Vector<double> MeasurementToCartesian(Measurement m)
        {
            double x = m.Range * Math.Cos(m.Elevation) * Math.Cos(m.Azimuth);
            double y = m.Range * Math.Cos(m.Elevation) * Math.Sin(m.Azimuth);
            double z = m.Range * Math.Sin(m.Elevation);
            return DenseVector.OfArray(new double[] { x, y, z });
        }

        #endregion

        #region Candidate Track Management

        private void CreateOrUpdateCandidates(List<Measurement> measurements, double[] beta0)
        {
            // Mark measurements that are well-associated with existing tracks.
            HashSet<int> associated = new HashSet<int>();
            for (int m = 0; m < measurements.Count; m++)
            {
                if (beta0[m] < 0.9)
                    associated.Add(m);
            }
            // For the remaining measurements, update or create new candidates.
            for (int m = 0; m < measurements.Count; m++)
            {
                if (associated.Contains(m))
                    continue;
                CreateOrUpdateCandidate(measurements[m]);
            }
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
            {
                bestCandidate.Measurements.Add(m);
            }
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

                // Create a new track using the 9-dimensional EKF.
                ITrackerFilter ekf = new ManeuveringEKF(initState, initCov, AccelNoise);

                JPDA_Track newTrack = new JPDA_Track
                {
                    TrackId = nextTrackId++,
                    Filter = ekf,
                    Age = 0,
                    ExistenceProb = 0.5
                };
                tracks.Add(newTrack);
                candidates.Remove(cand);
            }
        }

        #endregion

        // --- Track Pruning ---
        private void PruneTracks()
        {
            tracks.RemoveAll(t => (t.ExistenceProb < PruneThreshold) || (t.Age > MaxTrackAge));
        }

        /// <summary>
        /// Returns the list of confirmed tracks.
        /// </summary>
        public List<JPDA_Track> GetTracks() => tracks;
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

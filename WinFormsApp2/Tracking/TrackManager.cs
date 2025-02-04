using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RealRadarSim.Models;

namespace RealRadarSim.Tracking
{
    public class TrackManager
    {
        private List<JPDA_Track> tracks = new List<JPDA_Track>();
        private int nextTrackId = 1;
        public double InitGateThreshold { get; set; } = 14.07;
        public int InitRequiredHits { get; set; } = 3;
        public int InitScanWindow { get; set; } = 3;
        public double InitPosStd { get; set; } = 200.0;
        public double InitVelStd { get; set; } = 100.0;
        public double GatingThreshold { get; set; } = 25.0;
        public double AccelNoise { get; set; } = 2.0;
        public double ProbDetection { get; set; } = 0.9;
        public double ProbSurvival { get; set; } = 0.995;
        public double PruneThreshold { get; set; } = 0.01;
        public double MaxTrackMergeDist { get; set; } = 800.0;
        public double MaxTrackAge { get; set; } = 40.0;
        public double CandidateMergeDistance { get; set; } = 1500.0;

        private List<CandidateTrack> candidates = new List<CandidateTrack>();
        private Random rng;

        public TrackManager(Random rng)
        {
            this.rng = rng;
        }

        public void UpdateTracks(List<Measurement> measurements, double dt)
        {
            // Predict existing tracks.
            foreach (var t in tracks)
            {
                t.Filter.Predict(dt);
                t.Age += dt;
                t.ExistenceProb *= ProbSurvival;
            }

            // Gating.
            List<List<int>> gateMatrix = new List<List<int>>(tracks.Count);
            for (int i = 0; i < tracks.Count; i++)
                gateMatrix.Add(new List<int>());

            for (int i = 0; i < tracks.Count; i++)
            {
                var track = tracks[i];
                double adaptiveThreshold = (track.ExistenceProb > 0.6) ? (GatingThreshold * 1.5) : GatingThreshold;
                for (int m = 0; m < measurements.Count; m++)
                {
                    double md2 = MahalanobisDistanceSq(track, measurements[m]);
                    if (md2 < adaptiveThreshold)
                        gateMatrix[i].Add(m);
                }
            }

            // Build associations.
            var measurementAssociations = new Dictionary<int, List<int>>();
            for (int m = 0; m < measurements.Count; m++)
                measurementAssociations[m] = new List<int>();

            for (int i = 0; i < tracks.Count; i++)
                foreach (int mIdx in gateMatrix[i])
                    measurementAssociations[mIdx].Add(i);

            var trackMeasLikelihood = new List<Dictionary<int, double>>();
            for (int i = 0; i < tracks.Count; i++)
                trackMeasLikelihood.Add(new Dictionary<int, double>());

            foreach (var kvp in measurementAssociations)
            {
                int measIdx = kvp.Key;
                var trkList = kvp.Value;
                if (trkList.Count == 0) continue;
                var zVec = ToVector(measurements[measIdx]);
                foreach (int i in trkList)
                {
                    double likelihood = ComputeMeasurementLikelihood(tracks[i], zVec);
                    trackMeasLikelihood[i][measIdx] = likelihood;
                }
            }

            var measurementLikelihoodSum = new Dictionary<int, double>();
            for (int m = 0; m < measurements.Count; m++)
                measurementLikelihoodSum[m] = 0.0;
            for (int i = 0; i < tracks.Count; i++)
                foreach (var pair in trackMeasLikelihood[i])
                    measurementLikelihoodSum[pair.Key] += pair.Value;

            var beta = new Dictionary<(int i, int m), double>();
            for (int i = 0; i < tracks.Count; i++)
                foreach (var pair in trackMeasLikelihood[i])
                {
                    int mIdx = pair.Key;
                    double numerator = pair.Value * ProbDetection;
                    double denominator = measurementLikelihoodSum[mIdx] * ProbDetection + 1e-9;
                    double b = numerator / denominator;
                    beta[(i, mIdx)] = b;
                }

            // Update tracks with measurements.
            for (int i = 0; i < tracks.Count; i++)
            {
                double sumBeta = 0.0;
                foreach (int mIdx in gateMatrix[i])
                    sumBeta += beta.GetValueOrDefault((i, mIdx), 0.0);

                if (sumBeta > 1e-9)
                {
                    Vector<double> zEff = DenseVector.Create(3, 0.0);
                    foreach (int mIdx in gateMatrix[i])
                    {
                        double b = beta.GetValueOrDefault((i, mIdx), 0.0);
                        var zVec = ToVector(measurements[mIdx]);
                        zEff += zVec * b;
                    }
                    zEff /= sumBeta;
                    tracks[i].Filter.Update(zEff);
                    tracks[i].Age = 0;
                    tracks[i].ExistenceProb = Math.Min(1.0, tracks[i].ExistenceProb + 0.22);
                }
            }

            // Rescue association for unassociated measurements.
            double rescueRadius = 2000.0;
            HashSet<int> associatedMeasurements = new HashSet<int>();
            for (int m = 0; m < measurements.Count; m++)
            {
                if (measurementAssociations[m].Count > 0)
                    associatedMeasurements.Add(m);
            }
            for (int m = 0; m < measurements.Count; m++)
            {
                if (!associatedMeasurements.Contains(m))
                {
                    var mPos = MeasurementToCartesian(measurements[m]);
                    foreach (var track in tracks)
                    {
                        if (track.ExistenceProb > 0.8)
                        {
                            var tPos = track.Filter.State.SubVector(0, 3);
                            if (EuclideanDistance(tPos, mPos) < rescueRadius)
                            {
                                var zVec = ToVector(measurements[m]);
                                track.Filter.Update(zVec);
                                track.Age = 0;
                                track.ExistenceProb = Math.Min(1.0, track.ExistenceProb + 0.1);
                                associatedMeasurements.Add(m);
                                break;
                            }
                        }
                    }
                }
            }

            // Form/update candidate tracks.
            for (int m = 0; m < measurements.Count; m++)
            {
                if (!associatedMeasurements.Contains(m))
                {
                    if (!IsNewTrackCandidate(measurements[m]))
                        CreateOrUpdateCandidate(measurements[m]);
                }
            }
            ConfirmCandidates();
            MergeCloseTracks();
            PruneTracks();
        }

        private Vector<double> MeasurementToCartesian(Measurement m)
        {
            double x = m.Range * Math.Cos(m.Elevation) * Math.Cos(m.Azimuth);
            double y = m.Range * Math.Cos(m.Elevation) * Math.Sin(m.Azimuth);
            double z = m.Range * Math.Sin(m.Elevation);
            return DenseVector.OfArray(new double[] { x, y, z });
        }

        private double EuclideanDistance(Vector<double> v1, Vector<double> v2)
        {
            return (v1 - v2).L2Norm();
        }

        private double MahalanobisDistanceSq(JPDA_Track t, Measurement m)
        {
            var zPred = t.Filter.H(t.Filter.State);
            var zMeas = ToVector(m);
            var y = zMeas - zPred;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);
            var S = t.Filter.S();
            var sInv = S.Inverse();
            double dist2 = (y.ToRowMatrix() * sInv * y.ToColumnMatrix())[0, 0];
            return dist2;
        }

        private double ComputeMeasurementLikelihood(JPDA_Track t, Vector<double> z)
        {
            var zPred = t.Filter.H(t.Filter.State);
            var y = z - zPred;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);
            var S = t.Filter.S();
            double det = S.Determinant();
            if (det < 1e-12) det = 1e-12;
            var sInv = S.Inverse();
            double dist2 = (y.ToRowMatrix() * sInv * y.ToColumnMatrix())[0, 0];
            double normFactor = 1.0 / (Math.Pow(2 * Math.PI, 1.5) * Math.Sqrt(det));
            double likelihood = normFactor * Math.Exp(-0.5 * dist2);
            return likelihood;
        }

        private Vector<double> ToVector(Measurement m)
        {
            return DenseVector.OfArray(new double[] { m.Range, m.Azimuth, m.Elevation });
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

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }

        private bool IsNewTrackCandidate(Measurement m)
        {
            double rescueRadius = 1000.0;
            var mPos = MeasurementToCartesian(m);
            foreach (var t in tracks)
            {
                var tPos = t.Filter.State.SubVector(0, 3);
                if (EuclideanDistance(tPos, mPos) < rescueRadius)
                    return true;
            }
            return false;
        }

        private void CreateOrUpdateCandidate(Measurement m)
        {
            double bestDist = double.MaxValue;
            CandidateTrack bestCand = null;
            foreach (var c in candidates)
            {
                var last = c.Measurements.Last();
                double d = SphericalDistance(m, last);
                if (d < CandidateMergeDistance)
                {
                    if (d < bestDist)
                    {
                        bestDist = d;
                        bestCand = c;
                    }
                }
            }
            if (bestCand != null)
                bestCand.Measurements.Add(m);
            else
            {
                var newCand = new CandidateTrack();
                newCand.Measurements.Add(m);
                candidates.Add(newCand);
            }
        }

        private void ConfirmCandidates()
        {
            var toConfirm = new List<CandidateTrack>();
            foreach (var c in candidates)
            {
                if (c.Measurements.Count >= InitRequiredHits)
                    toConfirm.Add(c);
            }
            foreach (var c in toConfirm)
            {
                Vector<double> zSum = DenseVector.Create(3, 0.0);
                foreach (var meas in c.Measurements)
                    zSum += ToVector(meas);
                Vector<double> zAvg = zSum / c.Measurements.Count;
                var (state, cov) = MeasurementToInitialState(zAvg);
                var ekf = new AdvancedEKF(state, cov, AccelNoise);
                var newTrack = new JPDA_Track
                {
                    TrackId = nextTrackId++,
                    Filter = ekf,
                    Age = 0,
                    ExistenceProb = 0.5
                };
                tracks.Add(newTrack);
                candidates.Remove(c);
            }
        }

        private (Vector<double> state, Matrix<double> cov) MeasurementToInitialState(Vector<double> meas)
        {
            double r = meas[0];
            double az = meas[1];
            double el = meas[2];
            double x = r * Math.Cos(el) * Math.Cos(az);
            double y = r * Math.Cos(el) * Math.Sin(az);
            double z = r * Math.Sin(el);
            var state = DenseVector.OfArray(new double[] { x, y, z, 0, 0, 0 });
            var cov = DenseMatrix.Create(6, 6, 0.0);
            cov[0, 0] = InitPosStd * InitPosStd;
            cov[1, 1] = InitPosStd * InitPosStd;
            cov[2, 2] = InitPosStd * InitPosStd;
            cov[3, 3] = InitVelStd * InitVelStd;
            cov[4, 4] = InitVelStd * InitVelStd;
            cov[5, 5] = InitVelStd * InitVelStd;
            return (state, cov);
        }

        private double TrackMahalanobisDistance(JPDA_Track t1, JPDA_Track t2)
        {
            var diff = t1.Filter.State.SubVector(0, 3) - t2.Filter.State.SubVector(0, 3);
            var covSum = t1.Filter.Covariance.SubMatrix(0, 3, 0, 3) + t2.Filter.Covariance.SubMatrix(0, 3, 0, 3);
            var invCov = covSum.Inverse();
            double md2 = (diff.ToRowMatrix() * invCov * diff.ToColumnMatrix())[0, 0];
            return Math.Sqrt(md2);
        }

        private void MergeCloseTracks()
        {
            double mergeThreshold = 7.0;
            var toRemove = new List<JPDA_Track>();
            for (int i = 0; i < tracks.Count; i++)
            {
                for (int j = i + 1; j < tracks.Count; j++)
                {
                    var t1 = tracks[i];
                    var t2 = tracks[j];
                    if (TrackMahalanobisDistance(t1, t2) < mergeThreshold)
                    {
                        t1.Filter.State = 0.5 * (t1.Filter.State + t2.Filter.State);
                        t1.Filter.Covariance = 0.5 * (t1.Filter.Covariance + t2.Filter.Covariance);
                        t1.ExistenceProb = Math.Max(t1.ExistenceProb, t2.ExistenceProb);
                        toRemove.Add(t2);
                    }
                }
            }
            tracks.RemoveAll(t => toRemove.Contains(t));
        }

        private void PruneTracks()
        {
            tracks.RemoveAll(t => (t.ExistenceProb < PruneThreshold) || (t.Age > MaxTrackAge));
        }

        public List<JPDA_Track> GetTracks() => tracks;
    }

    public class CandidateTrack
    {
        public List<Measurement> Measurements = new List<Measurement>();
    }

    public class JPDA_Track
    {
        public int TrackId;
        public AdvancedEKF Filter;
        public double Age;
        public double ExistenceProb;
        public string FlightName;
    }
}

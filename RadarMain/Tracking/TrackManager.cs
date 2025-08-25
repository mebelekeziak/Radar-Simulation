using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
using RealRadarSim.Models;
using RealRadarSim.Utils;

namespace RealRadarSim.Tracking
{
    /// <summary>
    /// Manages tracks and performs the Joint Probabilistic Data Association (JPDA)
    /// update using adaptive gating, candidate track initiation, merging, and pruning.
    /// </summary>
    public class TrackManager
    {

        // ---------------------------------------------------------------------
        //  Loose‑gate rescue for “lost” fast movers
        // ---------------------------------------------------------------------
        private const double RescueGateChi2 = 25.0;   // χ²(3 DOF) ≈ 99.96% coverage

        /// <summary>
        /// Returns true if measurement m still fits track trk under the loose gate.
        /// Uses only the 3‑D position covariance.
        /// </summary>
        private bool FitsLoosePositionGate(Measurement m, JPDA_Track trk)
        {
            // Convert spherical meas → Cartesian
            var z = MeasurementToCartesian(m);        // Vector<double>[px,py,pz]
            var x = trk.Filter.State.SubVector(0, 3);

            var diff = z - x;
            var Ppos = trk.Filter.Covariance.SubMatrix(0, 3, 0, 3).Clone();
            for (int i = 0; i < 3; i++) Ppos[i, i] += 1e-6;  // stabilize

            double d2 = MahalanobisSquared(diff, Ppos);
            return d2 < RescueGateChi2;
        }

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

        /// Percentage of slant range that we are willing to fuse at long range.
        /// 0.03 → 3 % of 60 km ≈ 1.8 km.
        public double RangeMergeFraction { get; set; } = 0.03;

        // Measurement dimension (range, azimuth, elevation)
        private int measurementDimension = 3;

        public TrackManager(Random rng)
        {
            this.rng = rng;
        }

        // ---------------------------------------------------------------------
        //  Duplicate‑track suppression helpers
        // ---------------------------------------------------------------------
        private const double BaseDuplicateGateThreshold = 16.27;          // χ²(3 dof, P=0.997)

        /// <summary>
        /// Returns true if <paramref name="initState"/> would be a duplicate of an
        /// already‑confirmed track (tested in position‑only space).
        /// </summary>
        private bool IsDuplicateTrack(Vector<double> initState, Matrix<double> initCov)
        {
            Vector<double> pos = initState.SubVector(0, 3);

            foreach (var trk in tracks)
            {
                // a) positional Mahalanobis distance
                Vector<double> d = pos - trk.Filter.State.SubVector(0, 3);
                Matrix<double> S = initCov.SubMatrix(0, 3, 0, 3)
                                   + trk.Filter.Covariance.SubMatrix(0, 3, 0, 3);
                for (int k = 0; k < 3; ++k) S[k, k] += 1e-6;
                double d2 = MahalanobisSquared(d, S);

                // b) compute the same dynamic & existence factors you use elsewhere
                double speed = trk.Filter.State.SubVector(3, 3).L2Norm();
                double dynamicFactor = 1.0 + 0.5 * (speed / 100.0);
                double existenceFactor = (trk.ExistenceProb > 0.7) ? 1.5 : 1.0;

                // c) adaptive threshold
                double threshold = BaseDuplicateGateThreshold * dynamicFactor * existenceFactor;

                if (d2 < threshold)
                    return true;
            }

            return false;
        }


        /// <summary>
        /// Main update loop to predict, associate, update, initiate, merge, and prune tracks.
        /// </summary>
        /// <param name="measurements">List of radar measurements (range, azimuth, elevation, amplitude).</param>
        /// <param name="dt">Time step since the last update (seconds).</param>
        public void UpdateTracks(List<Measurement> measurements, double dt)
        {
            // 1) Prediction Step
            foreach (var trk in tracks)
            {
                trk.Filter.Predict(dt);
                trk.Age += dt;
            }

            int nTracks = tracks.Count;
            int nMeas = measurements.Count;

            // --- prepare reusable containers ---
            var gateMatrix = new List<List<int>>(nTracks);
            var measToTracks = new Dictionary<int, List<int>>(nMeas);
            var trackMeasLikelihood = new List<Dictionary<int, double>>(nTracks);
            double[] measurementLikelihoodSum = new double[nMeas];
            double[] beta0 = new double[nMeas];
            var beta = new Dictionary<(int, int), double>();

            // --- per‐track caches ---
            var zPreds = new Vector<double>[nTracks];
            var SInvs = new Matrix<double>[nTracks];
            var normFactors = new double[nTracks];
            var rangeGates = new double[nTracks];
            var sqrtDets = new double[nTracks];

            // --- precompute measurement vectors to avoid repeated boxing ---
            var zMeasVecs = new Vector<double>[nMeas];
            for (int m = 0; m < nMeas; m++)
            {
                var meas = measurements[m];
                zMeasVecs[m] = DenseVector.OfArray(new[] { meas.Range, meas.Azimuth, meas.Elevation });
            }

            // base chi‐square threshold
            var chiSq = new ChiSquared(measurementDimension);
            double baseGateTh = chiSq.InverseCumulativeDistribution(GatingProbability);

            // 2) Precompute H(x), S⁻¹, normalization and a cheap range‐gate per track (parallel)
            Parallel.For(0, nTracks, i =>
            {
                var trk = tracks[i];
                var zPred = trk.Filter.H(trk.Filter.State);
                zPreds[i] = zPred;

                var S = trk.Filter.S();
                // Stable inversion via Cholesky
                var chol = S.Cholesky();
                SInvs[i] = chol.Solve(DenseMatrix.CreateIdentity(S.RowCount));
                // sqrt(det(S)) from chol diag product
                var R = chol.Factor;
                double sqrtDet = 1.0; for (int k = 0; k < R.RowCount; k++) sqrtDet *= R[k, k];
                sqrtDets[i] = Math.Max(sqrtDet, 1e-6);
                normFactors[i] = 1.0 / (Math.Pow(2 * Math.PI, measurementDimension / 2.0) * sqrtDets[i]);

                // range‐only pre‐gate threshold: sqrt(χ² ⋅ var_range)
                rangeGates[i] = Math.Sqrt(baseGateTh * S[0, 0]);
            });

            // initialize gateMatrix
            for (int i = 0; i < nTracks; i++)
                gateMatrix.Add(new List<int>());

            // 3) Gating: cheap range‐gate + full Mahalanobis with cached S⁻¹ (parallel)
            Parallel.For(0, nTracks, i =>
            {
                var trk = tracks[i];
                double speed = trk.Filter.State.SubVector(3, 3).L2Norm();
                double dynFact = 1.0 + 0.5 * (speed / 100.0);
                double existFact = trk.ExistenceProb > 0.7 ? 1.5 : 1.0;
                double adaptTh = Math.Max(5.0,
                                    Math.Min(baseGateTh * dynFact * existFact, 1e6));

                for (int m = 0; m < nMeas; m++)
                {
                    var meas = measurements[m];

                    // 3.a) cheap range‐only pre‐gate
                    if (Math.Abs(meas.Range - zPreds[i][0]) > rangeGates[i])
                        continue;

                    // 3.b) full Mahalanobis
                    var y = zMeasVecs[m] - zPreds[i];
                    y[1] = MathUtil.NormalizeAngle(y[1]);
                    y[2] = MathUtil.NormalizeAngle(y[2]);

                    double d2 = (y.ToRowMatrix() * SInvs[i] * y.ToColumnMatrix())[0, 0];

                    // SNR-adaptive gating: tighten threshold for strong detections
                    double snrLin = Math.Max(0.0, Math.Pow(10.0, meas.SNR_dB / 10.0));
                    double snrGateFactor = 1.0 / (1.0 + 0.3 * snrLin); // in (0,1] as SNR grows
                    double adaptThSNR = Math.Max(2.0, adaptTh * Math.Clamp(snrGateFactor, 0.2, 1.0));

                    if (d2 < adaptThSNR)
                        gateMatrix[i].Add(m);
                }
            });

            // 4) Build measurement→track lists
            for (int m = 0; m < nMeas; m++)
                measToTracks[m] = new List<int>();
            for (int i = 0; i < nTracks; i++)
                foreach (int m in gateMatrix[i])
                    measToTracks[m].Add(i);

            // 5) Compute likelihoods using cached normFactors and SInvs (parallel)
            for (int i = 0; i < nTracks; i++)
                trackMeasLikelihood.Add(new Dictionary<int, double>());
            Parallel.For(0, nMeas, m =>
            {
                var trkList = measToTracks[m];
                if (trkList.Count == 0) return;

                var z = zMeasVecs[m];
                foreach (int i in trkList)
                {
                    var y = z - zPreds[i];
                    y[1] = MathUtil.NormalizeAngle(y[1]);
                    y[2] = MathUtil.NormalizeAngle(y[2]);

                    double d2 = (y.ToRowMatrix() * SInvs[i] * y.ToColumnMatrix())[0, 0];
                    double L = normFactors[i] * Math.Exp(-0.5 * d2);
                    // Slightly emphasize high-SNR measurements in association
                    double snrLin = Math.Max(0.0, Math.Pow(10.0, measurements[m].SNR_dB / 10.0));
                    double confBoost = Math.Clamp(Math.Sqrt(1.0 + snrLin), 1.0, 4.0);
                    L *= confBoost;

                    lock (trackMeasLikelihood[i])
                        trackMeasLikelihood[i][m] = L;
                }
            });

            // 6) Sum likelihoods over tracks
            for (int m = 0; m < nMeas; m++)
            {
                double sum = 0.0;
                for (int i = 0; i < nTracks; i++)
                    if (trackMeasLikelihood[i].TryGetValue(m, out var L))
                        sum += ProbDetection * L;
                measurementLikelihoodSum[m] = sum;
            }

            // 7) Compute β₀ (false‐alarm) and β(i,m) (association probs)
            for (int m = 0; m < nMeas; m++)
            {
                double baseVol = (4.0 / 3.0) * Math.PI * Math.Pow(Math.Sqrt(baseGateTh), 3);
                var trkList = measToTracks[m];
                double avgSqrtDet = 1.0;
                if (trkList.Count > 0)
                {
                    double tot = 0.0;
                    foreach (int i in trkList)
                        tot += sqrtDets[i];
                    avgSqrtDet = tot / trkList.Count;
                }

                double clutterL = ClutterDensity * baseVol * avgSqrtDet;
                double totalL = measurementLikelihoodSum[m] + clutterL;
                if (totalL < 1e-12) totalL = 1e-12;
                beta0[m] = clutterL / totalL;

                foreach (int i in trkList)
                    beta[(i, m)] = (ProbDetection * trackMeasLikelihood[i][m]) / totalL;
            }

            // 8) Apply JPDA update per track using association probabilities
            for (int i = 0; i < nTracks; i++)
            {
                double wsum = 0.0;
                int bestM = -1;
                double bestW = -1.0;
                foreach (int m in gateMatrix[i])
                {
                    if (beta.TryGetValue((i, m), out var w))
                    {
                        wsum += w;
                        if (w > bestW)
                        {
                            bestW = w;
                            bestM = m;
                        }
                    }
                }

                if (wsum > 1e-6 && bestM >= 0)
                {
                    var trk = tracks[i];
                    var z = zMeasVecs[bestM];
                    var baseR = trk.Filter.GetMeasurementNoiseCov().Clone();
                    double snrLin = Math.Max(1e-6, Math.Pow(10.0, measurements[bestM].SNR_dB / 10.0));
                    double scale = 1.0 / Math.Sqrt(1.0 + snrLin);
                    baseR[0, 0] *= scale;
                    baseR[1, 1] *= scale;
                    baseR[2, 2] *= scale;

                    trk.Filter.Update(z, baseR);
                    trk.Age = 0;
                    double incr = Math.Clamp(wsum, 0.1, 1.0) * ExistenceIncreaseGain;
                    trk.ExistenceProb = Math.Min(1.0, trk.ExistenceProb + incr);
                }
                else
                {
                    tracks[i].ExistenceProb *= ExistenceDecayFactor;
                }
            }

            CreateOrUpdateCandidates(measurements, beta0);
            ConfirmCandidates();
            MergeTracksDBSCAN();
            PruneTracks();
        }

        #region DBSCAN-Based Track Merging Methods
        /* ---------------------------------------------------------------------------
         * Uncertainty–aware DBSCAN + Information–space fusion
         * ---------------------------------------------------------------------------
         *  • Distance metric: 3‑D Mahalanobis distance that accounts for the summed
         *    position covariance of the two tracks      d² = Δxᵀ (P₁ₚ + P₂ₚ)⁻¹ Δx
         *  • Threshold: χ² inv‑cdf for 3 DOF at the user‑tunable GatingProbability
         *    (defaults to 0.997 → 16.27).  A hard Euclidean guard
         *    (MaxTrackMergeDist²) is also applied to avoid fusing far–apart targets.
         *  • Fusion: true information‑filter fusion performed **once per cluster**
         *    (∑P⁻¹, ∑P⁻¹x).  Keeps numerics stable and avoids order–dependent drift.
         *  • Existence probability: takes the maximum over the cluster, preserving
         *    the strongest belief.
         * -------------------------------------------------------------------------*/

        /// <summary>
        /// Cluster confirmed tracks with an uncertainty‑aware DBSCAN and fuse
        /// every cluster into a single Gaussian track in information space.
        /// </summary>
        private void MergeTracksDBSCAN()
        {
            int n = tracks.Count;
            if (n < 2)           // nothing to do
                return;

            /* ----------  pre‑compute position means & covariances  ---------- */
            var pos = new Vector<double>[n];      // Cartesian position (3×1)
            var posCov = new Matrix<double>[n];      // Position cov. (3×3)
            for (int i = 0; i < n; i++)
            {
                pos[i] = tracks[i].Filter.State.SubVector(0, 3);
                posCov[i] = tracks[i].Filter.Covariance.SubMatrix(0, 3, 0, 3);
            }

            /* ----------  DBSCAN parameters  ---------- */
            double epsMahalanobisSq = new ChiSquared(3)
                .InverseCumulativeDistribution(GatingProbability);        // χ² threshold
            double maxEuclidSq = MaxTrackMergeDist * MaxTrackMergeDist;   // hard guard
            int minPts = 2;                                       // cluster size

            /* ----------  DBSCAN main loop  ---------- */
            int[] clusterIds = Enumerable.Repeat(-1, n).ToArray();
            bool[] visited = new bool[n];
            int clusterId = 0;

            for (int i = 0; i < n; i++)
            {
                if (visited[i]) continue;
                visited[i] = true;

                var neighbors = RegionQueryMahalanobis(pos, posCov, i,
                                                       epsMahalanobisSq, maxEuclidSq);

                if (neighbors.Count < minPts)
                {
                    clusterIds[i] = 0;          // noise
                    continue;
                }

                clusterId++;
                ExpandClusterMahalanobis(i, neighbors, clusterId,
                                         epsMahalanobisSq, maxEuclidSq,
                                         visited, clusterIds, pos, posCov);
            }

            /* ----------  fuse each cluster in information space  ---------- */
            var clusters = new Dictionary<int, List<JPDA_Track>>();
            for (int i = 0; i < n; i++)
            {
                int cid = clusterIds[i];
                if (!clusters.ContainsKey(cid))
                    clusters[cid] = new List<JPDA_Track>();
                clusters[cid].Add(tracks[i]);
            }

            var mergedTracks = new List<JPDA_Track>();
            foreach (var kv in clusters)
            {
                if (kv.Key == 0)                     // clusterId 0 = noise → keep as‑is
                {
                    mergedTracks.AddRange(kv.Value);
                    continue;
                }

                var cl = kv.Value;                   // ≥ 2 tracks guaranteed (minPts=2)
                mergedTracks.Add(FuseTrackCluster(cl));
            }
            tracks = mergedTracks;
        }

        /* ------------------------------------------------------------------ */
        /* ---------------------  helper functions  ------------------------- */
        /* ------------------------------------------------------------------ */

        /// Return neighbour indices whose Mahalanobis distance to “index”
        /// is below χ²(3) and whose Euclidean spacing is not absurdly large.
        /// The Euclidean guard adapts with range:  max( MaxTrackMergeDist ,
        /// RangeMergeFraction × slant‑range ).
        private List<int> RegionQueryMahalanobis(
            Vector<double>[] pos,
            Matrix<double>[] posCov,
            int index,
            double epsMahalanobisSq,
            double maxEuclidSq /* retained but now used only at short range */)
        {
            var neighbours = new List<int>();

            // dynamic Euclidean guard – 3 % of the slant range
            double r = pos[index].L2Norm();
            double dynGuardSq = (RangeMergeFraction * r) * (RangeMergeFraction * r);

            double guardSq = Math.Max(maxEuclidSq, dynGuardSq);  // pick the larger

            for (int j = 0; j < pos.Length; ++j)
            {
                Vector<double> d = pos[j] - pos[index];
                double euclidSq = d.DotProduct(d);
                if (euclidSq > guardSq) continue;                 // quick reject

                // Mahalanobis test using a Cholesky solve (avoids explicit inverse)
                Matrix<double> S = posCov[index] + posCov[j];
                for (int k = 0; k < 3; ++k) S[k, k] += 1e-6;
                double d2 = MahalanobisSquared(d, S);

                if (d2 <= epsMahalanobisSq)
                    neighbours.Add(j);
            }
            return neighbours;
        }


        /// <summary>
        /// Recursive region growth for DBSCAN (Mahalanobis metric).
        /// </summary>
        private void ExpandClusterMahalanobis(
            int index,
            List<int> neighborIndices,
            int clusterId,
            double epsMahalanobisSq,
            double maxEuclidSq,
            bool[] visited,
            int[] clusterIds,
            Vector<double>[] pos,
            Matrix<double>[] posCov)
        {
            clusterIds[index] = clusterId;
            var seeds = new Queue<int>(neighborIndices);

            while (seeds.Count > 0)
            {
                int current = seeds.Dequeue();

                if (!visited[current])
                {
                    visited[current] = true;
                    var currentNeighbors = RegionQueryMahalanobis(
                                               pos, posCov, current,
                                               epsMahalanobisSq, maxEuclidSq);

                    if (currentNeighbors.Count > 0)          // minPts = 1
                    {
                        foreach (var n in currentNeighbors)
                            if (!seeds.Contains(n))
                                seeds.Enqueue(n);
                    }
                }

                if (clusterIds[current] == -1)
                    clusterIds[current] = clusterId;
            }
        }

        /// <summary>
        /// Fuse all tracks in a cluster in information space (∑P⁻¹, ∑P⁻¹x).
        /// Numerical order no longer matters and the result is the ML Gaussian.
        /// </summary>
        private JPDA_Track FuseTrackCluster(List<JPDA_Track> cluster)
        {
            /* accumulate information matrices/vectors */
            Matrix<double> Omega = DenseMatrix.Create(9, 9, 0.0);   // ∑P⁻¹
            Vector<double> xi = DenseVector.Create(9, 0.0);      // ∑P⁻¹ x
            double maxExistProb = 0.0;

            foreach (var trk in cluster)
            {
                // Use Cholesky factorization for a stable inverse
                var chol = trk.Filter.Covariance.Cholesky();
                Matrix<double> PInv = chol.Solve(DenseMatrix.CreateIdentity(9));
                Omega += PInv;
                // P^{-1} x can be obtained without forming P^{-1}, but we already have PInv here
                xi += PInv * trk.Filter.State;
                if (trk.ExistenceProb > maxExistProb)
                    maxExistProb = trk.ExistenceProb;
            }

            Matrix<double> fusedCov = Omega.Inverse();
            Vector<double> fusedState = fusedCov * xi;

            /* write back into a representative track (first in the list) */
            JPDA_Track rep = cluster[0];
            rep.Filter.State = fusedState;
            rep.Filter.Covariance = fusedCov;
            rep.ExistenceProb = maxExistProb;
            return rep;
        }
        #endregion


        #region Helper Methods

        /// <summary>
        /// Computes d^T S^{-1} d using a Cholesky solve for numerical stability.
        /// </summary>
        private static double MahalanobisSquared(Vector<double> d, Matrix<double> S)
        {
            var chol = S.Cholesky();
            // Solve S x = d
            Vector<double> x = chol.Solve(d);
            return d.DotProduct(x);
        }

        /// <summary>
        /// Computes the Mahalanobis distance squared between the predicted measurement and the actual measurement.
        /// </summary>
        private double MahalanobisDistanceSq(JPDA_Track track, Measurement m)
        {
            Vector<double> zPred = track.Filter.H(track.Filter.State);
            Vector<double> zMeas = ToVector(m);
            Vector<double> y = zMeas - zPred;
            y[1] = MathUtil.NormalizeAngle(y[1]);
            y[2] = MathUtil.NormalizeAngle(y[2]);
            Matrix<double> S = track.Filter.S();
            // tiny jitter for stability
            for (int i = 0; i < S.RowCount; i++) S[i, i] += 1e-9;
            return MahalanobisSquared(y, S);
        }

        /// <summary>
        /// Computes the measurement likelihood for a given track and measurement.
        /// </summary>
        private double ComputeMeasurementLikelihood(JPDA_Track track, Vector<double> z)
        {
            Vector<double> zPred = track.Filter.H(track.Filter.State);
            Vector<double> y = z - zPred;
            y[1] = MathUtil.NormalizeAngle(y[1]);
            y[2] = MathUtil.NormalizeAngle(y[2]);

            Matrix<double> S = track.Filter.S();
            for (int i = 0; i < S.RowCount; i++) S[i, i] += 1e-9;
            var chol = S.Cholesky();
            // sqrt(det(S)) is product of Cholesky diagonal
            var R = chol.Factor;
            double sqrtDet = 1.0; for (int i = 0; i < R.RowCount; i++) sqrtDet *= R[i, i];
            double dist2 = MahalanobisSquared(y, S);
            double normFactor = 1.0 / (Math.Pow(2 * Math.PI, measurementDimension / 2.0) * Math.Max(sqrtDet, 1e-6));
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
            // Convert to the 3‑D vector 
            Vector<double> measVec = DenseVector.OfArray(new double[] { m.Range, m.Azimuth, m.Elevation });
            // Build a 9×9 cov with your same initial STD settings
            var (initState, initCov) = MeasurementToInitialState(measVec);
            if (IsDuplicateTrack(initState, initCov))
                return;   // right here — no candidate ever created

            /* -------------------------------------------------------------
             * 1)  FIRST: can this measurement still belong to an EXISTING
             *     track under a *loose* 3‑σ position gate?  If yes we feed
             *     it into that track and STOP — no candidate needed.
             * ----------------------------------------------------------- */
            foreach (var trk in tracks)
            {
                if (FitsLoosePositionGate(m, trk))
                {
                    // Convert into the 3‑D spherical vector expected by the EKF
                    Vector<double> z = ToVector(m);

                    // Adaptive R based on measurement SNR (higher SNR → lower noise)
                    Matrix<double> baseR = trk.Filter.GetMeasurementNoiseCov().Clone();
                    double snrLin = Math.Max(1e-6, Math.Pow(10.0, m.SNR_dB / 10.0));
                    double scale = 1.0 / Math.Sqrt(1.0 + snrLin);
                    baseR[0, 0] *= scale;
                    baseR[1, 1] *= scale;
                    baseR[2, 2] *= scale;

                    trk.Filter.Update(z, baseR);

                    // House‑keeping
                    trk.Age = 0;
                    trk.ExistenceProb = Math.Min(1.0, trk.ExistenceProb + ExistenceIncreaseGain);
                    return;                         // measurement consumed – we're done
                }
            }

            /* -------------------------------------------------------------
             * 2) Otherwise proceed with the classic candidate logic
             * ----------------------------------------------------------- */

            // Bail out if still too close to an existing track *geometrically*
            foreach (var trk in tracks)
                if (MeasurementTrackDistance(m, trk) < CandidateMergeDistance)
                    return;

            // Merge with an existing candidate if close enough, else open a new one
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

        // Quick 3D distance between a raw measurement and a confirmed track
        private double MeasurementTrackDistance(Measurement m, JPDA_Track track)
        {
            // track.Filter.State holds [px, py, pz, vx, vy, vz, ax, ay, az]
            var p = track.Filter.State;
            double xT = p[0], yT = p[1], zT = p[2];

            // Convert spherical measurement to Cartesian
            double xM = m.Range * Math.Cos(m.Elevation) * Math.Cos(m.Azimuth);
            double yM = m.Range * Math.Cos(m.Elevation) * Math.Sin(m.Azimuth);
            double zM = m.Range * Math.Sin(m.Elevation);

            double dx = xM - xT;
            double dy = yM - yT;
            double dz = zM - zT;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }


        /// <summary>
        /// Confirm candidate tracks once they have enough hits.
        /// </summary>
        private void ConfirmCandidates()
        {
            // 1) Gather candidates ready for confirmation
            List<CandidateTrack> toConfirm = new List<CandidateTrack>();
            foreach (var cand in candidates)
            {
                if (cand.Measurements.Count >= InitRequiredHits)
                    toConfirm.Add(cand);
            }

            // 2) For each, compute its initial state & covariance,
            //    reject duplicates, then promote to a real track.
            foreach (var cand in toConfirm)
            {
                // a) Compute average measurement
                Vector<double> zSum = DenseVector.Create(3, 0.0);
                foreach (var meas in cand.Measurements)
                    zSum += ToVector(meas);
                Vector<double> zAvg = zSum / cand.Measurements.Count;

                // b) Build initial state & cov
                (Vector<double> initState, Matrix<double> initCov) = MeasurementToInitialState(zAvg);

                // -------------------------------------------------------------
                //  <<< NEW ­– duplicate‑track suppression >>>
                // -------------------------------------------------------------
                if (IsDuplicateTrack(initState, initCov))
                {
                    // A valid track already covers this region – discard candidate.
                    candidates.Remove(cand);
                    continue;
                }
                // -------------------------------------------------------------

                // c) Create EKF & new JPDA_Track
                ITrackerFilter ekf = new ManeuveringEKF(initState, initCov, AccelNoise);

                JPDA_Track newTrack = new JPDA_Track
                {
                    TrackId = nextTrackId++,
                    Filter = ekf,
                    Age = 0,
                    ExistenceProb = 0.5
                };
                tracks.Add(newTrack);

                // d) Remove from candidates list
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

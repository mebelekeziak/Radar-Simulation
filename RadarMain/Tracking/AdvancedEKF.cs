using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RealRadarSim.Tracking
{
    #region ITrackerFilter Interface
    /// <summary>
    /// Interface for tracking filters.
    /// </summary>
    public interface ITrackerFilter
    {
        Vector<double> State { get; set; }
        Matrix<double> Covariance { get; set; }
        void Predict(double dt);
        void Update(Vector<double> z);
        /// <summary>
        /// New update overload that accepts a custom measurement covariance.
        /// </summary>
        void Update(Vector<double> z, Matrix<double> customMeasurementCov);
        /// <summary>
        /// Nonlinear measurement function mapping state to measurement space.
        /// </summary>
        Vector<double> H(Vector<double> x);
        /// <summary>
        /// Returns the innovation (measurement) covariance.
        /// </summary>
        Matrix<double> S();
        /// <summary>
        /// Returns the filter’s intrinsic measurement noise covariance.
        /// </summary>
        Matrix<double> GetMeasurementNoiseCov();
    }
    #endregion

    #region EKFFilter (Improved EKF with Adaptive Process Noise)
    /// <summary>
    /// Improved Extended Kalman Filter with adaptive process noise.
    /// </summary>
    public class EKFFilter : ITrackerFilter
    {
        public Vector<double> State { get; set; }       // [px, py, pz, vx, vy, vz]
        public Matrix<double> Covariance { get; set; }    // 6x6

        private double sigmaA;         // nominal acceleration noise
        private double adaptiveSigmaA; // adaptive acceleration noise used for Q
        private Matrix<double> R;      // measurement noise covariance (3x3)

        public EKFFilter(Vector<double> initState, Matrix<double> initCov, double accelNoise)
        {
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaA = accelNoise;
            adaptiveSigmaA = sigmaA; // initialize adaptive noise to nominal value

            // Example measurement noise: range std dev = 50, azimuth & elevation std dev = 0.001.
            R = DenseMatrix.OfArray(new double[,]
            {
                { 50 * 50,      0,              0 },
                { 0,        0.001 * 0.001,      0 },
                { 0,             0,        0.001 * 0.001 }
            });
        }

        public void Predict(double dt)
        {
            // Constant-velocity state transition.
            var F = DenseMatrix.CreateIdentity(6);
            F[0, 3] = dt;
            F[1, 4] = dt;
            F[2, 5] = dt;
            State = F * State;

            // Process noise Q (from random acceleration) using adaptiveSigmaA.
            double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
            double q = adaptiveSigmaA * adaptiveSigmaA;
            var Q = DenseMatrix.Create(6, 6, 0.0);
            Q[0, 0] = dt4 / 4.0 * q; Q[0, 3] = dt3 / 2.0 * q;
            Q[1, 1] = dt4 / 4.0 * q; Q[1, 4] = dt3 / 2.0 * q;
            Q[2, 2] = dt4 / 4.0 * q; Q[2, 5] = dt3 / 2.0 * q;
            Q[3, 0] = dt3 / 2.0 * q;
            Q[4, 1] = dt3 / 2.0 * q;
            Q[5, 2] = dt3 / 2.0 * q;
            Q[3, 3] = dt2 * q;
            Q[4, 4] = dt2 * q;
            Q[5, 5] = dt2 * q;

            Covariance = F * Covariance * F.Transpose() + Q;
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        /// <summary>
        /// Standard EKF update using the filter’s default measurement noise.
        /// </summary>
        public void Update(Vector<double> z)
        {
            Update(z, R);
        }

        /// <summary>
        /// Update overload that uses a custom measurement covariance (e.g. from a JPDA combined update).
        /// </summary>
        public void Update(Vector<double> z, Matrix<double> customMeasurementCov)
        {
            // Compute measurement prediction h(x) and innovation.
            Vector<double> hVal = H(State);
            Vector<double> y = z - hVal;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);

            // Compute Jacobian of the measurement function.
            Matrix<double> Hjac = Jacobian(State);
            // Compute innovation covariance using the custom measurement covariance.
            Matrix<double> S_mat = Hjac * Covariance * Hjac.Transpose() + customMeasurementCov;
            var cholS = S_mat.Cholesky();
            Matrix<double> Sinv = cholS.Solve(DenseMatrix.CreateIdentity(3));

            // --- Adaptive Process Noise Update ---
            // Compute innovation Mahalanobis distance.
            double md2 = (y.ToRowMatrix() * Sinv * y.ToColumnMatrix())[0, 0];
            if (md2 > 9.0)
                adaptiveSigmaA = Math.Min(adaptiveSigmaA * 1.05, sigmaA * 5.0);
            else if (md2 < 3.0)
                adaptiveSigmaA = Math.Max(adaptiveSigmaA * 0.95, sigmaA);

            // Compute Kalman gain.
            Matrix<double> K = Covariance * Hjac.Transpose() * Sinv;
            // Update state.
            State = State + K * y;
            var I = DenseMatrix.CreateIdentity(6);
            Matrix<double> KH = K * Hjac;
            // Update covariance.
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K * customMeasurementCov * K.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        public Vector<double> H(Vector<double> x)
        {
            // Measurement function: returns [range, azimuth, elevation]
            double px = x[0], py = x[1], pz = x[2];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double az = Math.Atan2(py, px);
            double rho = Math.Sqrt(px * px + py * py);
            if (rho < 1e-9) rho = 1e-9;
            double el = Math.Atan2(pz, rho);
            return DenseVector.OfArray(new double[] { range, az, el });
        }

        public Matrix<double> S()
        {
            Matrix<double> Hjac = Jacobian(State);
            return Hjac * Covariance * Hjac.Transpose() + R;
        }

        public Matrix<double> GetMeasurementNoiseCov()
        {
            return R;
        }

        // Computes the Jacobian of the measurement function.
        private Matrix<double> Jacobian(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double denomXY = px * px + py * py;
            if (denomXY < 1e-9) denomXY = 1e-9;
            double rho = Math.Sqrt(denomXY);
            if (rho < 1e-9) rho = 1e-9;

            var H = DenseMatrix.Create(3, 6, 0.0);
            // Partial derivatives for range.
            H[0, 0] = px / range; H[0, 1] = py / range; H[0, 2] = pz / range;
            // Partial derivatives for azimuth.
            H[1, 0] = -py / denomXY; H[1, 1] = px / denomXY;
            // Partial derivatives for elevation.
            double range2 = range * range;
            H[2, 0] = -pz * px / (range2 * rho);
            H[2, 1] = -pz * py / (range2 * rho);
            H[2, 2] = rho / range2;
            return H;
        }

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI)
                angle -= 2.0 * Math.PI;
            while (angle < -Math.PI)
                angle += 2.0 * Math.PI;
            return angle;
        }
    }
    #endregion

    #region UKFFilter Implementation (Improved UKF with Adaptive Process Noise)
    /// <summary>
    /// Unscented Kalman Filter implementation with adaptive process noise.
    /// </summary>
    public class UKFFilter : ITrackerFilter
    {
        public Vector<double> State { get; set; }       // [px, py, pz, vx, vy, vz]
        public Matrix<double> Covariance { get; set; }    // 6x6

        private double sigmaA;
        private double adaptiveSigmaA; // adaptive acceleration noise used for Q
        private Matrix<double> R;      // measurement noise covariance (3x3)
        private int n;                 // state dimension (6)
        private double alpha, beta, kappa, lambda;      // UKF tuning parameters

        public UKFFilter(Vector<double> initState, Matrix<double> initCov, double accelNoise,
                         double rangeNoise = 50.0, double azStd = 0.001, double elStd = 0.001,
                         double alpha = 1e-1, double beta = 2.0, double kappa = 0.0)
        {
            n = 6;
            this.alpha = alpha;
            this.beta = beta;
            this.kappa = kappa;
            lambda = alpha * alpha * (n + kappa) - n;
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaA = accelNoise;
            adaptiveSigmaA = sigmaA; // initialize adaptive noise to nominal value

            R = DenseMatrix.OfArray(new double[,]
            {
                { rangeNoise * rangeNoise,   0,                      0 },
                { 0,                    azStd * azStd,               0 },
                { 0,                           0,              elStd * elStd }
            });
        }

        public void Predict(double dt)
        {
            Vector<double>[] sigmaPoints = GenerateSigmaPoints(State, Covariance);
            Vector<double>[] predictedSigmaPoints = new Vector<double>[2 * n + 1];
            for (int i = 0; i < sigmaPoints.Length; i++)
            {
                predictedSigmaPoints[i] = ProcessModel(sigmaPoints[i], dt);
            }
            double[] wm = GetWeightsMean();
            double[] wc = GetWeightsCov();
            Vector<double> xPred = DenseVector.Create(n, 0.0);
            for (int i = 0; i < predictedSigmaPoints.Length; i++)
            {
                xPred += wm[i] * predictedSigmaPoints[i];
            }
            Matrix<double> PPred = DenseMatrix.Create(n, n, 0.0);
            for (int i = 0; i < predictedSigmaPoints.Length; i++)
            {
                var diff = predictedSigmaPoints[i] - xPred;
                PPred += wc[i] * diff.OuterProduct(diff);
            }
            // Process noise Q computed with adaptiveSigmaA.
            Matrix<double> Q = ComputeProcessCovariance(dt, adaptiveSigmaA);
            PPred += Q;
            State = xPred;
            Covariance = PPred;
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        /// <summary>
        /// Standard UKF update using the default measurement noise.
        /// </summary>
        public void Update(Vector<double> z)
        {
            Update(z, R);
        }

        /// <summary>
        /// UKF update overload that accepts a custom measurement covariance.
        /// </summary>
        public void Update(Vector<double> z, Matrix<double> customMeasurementCov)
        {
            Vector<double>[] sigmaPoints = GenerateSigmaPoints(State, Covariance);
            Vector<double>[] zSigmaPoints = new Vector<double>[2 * n + 1];
            for (int i = 0; i < sigmaPoints.Length; i++)
            {
                zSigmaPoints[i] = MeasurementModel(sigmaPoints[i]);
            }
            double[] wm = GetWeightsMean();
            double[] wc = GetWeightsCov();
            Vector<double> zPred = DenseVector.Create(3, 0.0);
            for (int i = 0; i < zSigmaPoints.Length; i++)
            {
                zPred += wm[i] * zSigmaPoints[i];
            }
            Matrix<double> S_mat = DenseMatrix.Create(3, 3, 0.0);
            Matrix<double> Pxz = DenseMatrix.Create(n, 3, 0.0);
            for (int i = 0; i < zSigmaPoints.Length; i++)
            {
                var zDiff = zSigmaPoints[i] - zPred;
                zDiff[1] = NormalizeAngle(zDiff[1]);
                zDiff[2] = NormalizeAngle(zDiff[2]);
                var xDiff = sigmaPoints[i] - State;
                S_mat += wc[i] * zDiff.OuterProduct(zDiff);
                Pxz += wc[i] * xDiff.OuterProduct(zDiff);
            }
            S_mat += customMeasurementCov;
            var cholS = S_mat.Cholesky();
            Matrix<double> SInv = cholS.Solve(DenseMatrix.CreateIdentity(3));
            Matrix<double> K = Pxz * SInv;
            Vector<double> zDiffMeas = z - zPred;
            zDiffMeas[1] = NormalizeAngle(zDiffMeas[1]);
            zDiffMeas[2] = NormalizeAngle(zDiffMeas[2]);

            // --- Adaptive Process Noise Update ---
            double md2 = (zDiffMeas.ToRowMatrix() * SInv * zDiffMeas.ToColumnMatrix())[0, 0];
            if (md2 > 9.0)
                adaptiveSigmaA = Math.Min(adaptiveSigmaA * 1.05, sigmaA * 5.0);
            else if (md2 < 3.0)
                adaptiveSigmaA = Math.Max(adaptiveSigmaA * 0.95, sigmaA);

            State = State + K * zDiffMeas;
            Covariance = Covariance - K * S_mat * K.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        public Vector<double> H(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double az = Math.Atan2(py, px);
            double rho = Math.Sqrt(px * px + py * py);
            if (rho < 1e-9) rho = 1e-9;
            double el = Math.Atan2(pz, rho);
            return DenseVector.OfArray(new double[] { range, az, el });
        }

        public Matrix<double> S()
        {
            Vector<double>[] sigmaPoints = GenerateSigmaPoints(State, Covariance);
            Vector<double>[] zSigmaPoints = new Vector<double>[2 * n + 1];
            for (int i = 0; i < sigmaPoints.Length; i++)
            {
                zSigmaPoints[i] = MeasurementModel(sigmaPoints[i]);
            }
            double[] wm = GetWeightsMean();
            double[] wc = GetWeightsCov();
            Vector<double> zPred = DenseVector.Create(3, 0.0);
            for (int i = 0; i < zSigmaPoints.Length; i++)
            {
                zPred += wm[i] * zSigmaPoints[i];
            }
            Matrix<double> S_mat = DenseMatrix.Create(3, 3, 0.0);
            for (int i = 0; i < zSigmaPoints.Length; i++)
            {
                var diff = zSigmaPoints[i] - zPred;
                diff[1] = NormalizeAngle(diff[1]);
                diff[2] = NormalizeAngle(diff[2]);
                S_mat += wc[i] * diff.OuterProduct(diff);
            }
            S_mat += R;
            return S_mat;
        }

        public Matrix<double> GetMeasurementNoiseCov()
        {
            return R;
        }

        #region UKF Utility Methods
        private Vector<double>[] GenerateSigmaPoints(Vector<double> x, Matrix<double> P)
        {
            Vector<double>[] sigmaPoints = new Vector<double>[2 * n + 1];
            Matrix<double> sqrtMatrix = (P * (n + lambda)).Cholesky().Factor;
            sigmaPoints[0] = x;
            for (int i = 0; i < n; i++)
            {
                var col = sqrtMatrix.Column(i);
                sigmaPoints[i + 1] = x + col;
                sigmaPoints[i + 1 + n] = x - col;
            }
            return sigmaPoints;
        }

        private double[] GetWeightsMean()
        {
            double[] weights = new double[2 * n + 1];
            weights[0] = lambda / (n + lambda);
            for (int i = 1; i < 2 * n + 1; i++)
            {
                weights[i] = 1.0 / (2.0 * (n + lambda));
            }
            return weights;
        }

        private double[] GetWeightsCov()
        {
            double[] weights = new double[2 * n + 1];
            weights[0] = lambda / (n + lambda) + (1 - alpha * alpha + beta);
            for (int i = 1; i < 2 * n + 1; i++)
            {
                weights[i] = 1.0 / (2.0 * (n + lambda));
            }
            return weights;
        }

        private Vector<double> ProcessModel(Vector<double> x, double dt)
        {
            double px = x[0], py = x[1], pz = x[2];
            double vx = x[3], vy = x[4], vz = x[5];
            double pxNew = px + vx * dt;
            double pyNew = py + vy * dt;
            double pzNew = pz + vz * dt;
            return DenseVector.OfArray(new double[] { pxNew, pyNew, pzNew, vx, vy, vz });
        }

        private Matrix<double> ComputeProcessCovariance(double dt, double accelNoise)
        {
            double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
            double q = accelNoise * accelNoise;
            var Q = DenseMatrix.Create(6, 6, 0.0);
            Q[0, 0] = dt4 / 4.0 * q; Q[0, 3] = dt3 / 2.0 * q;
            Q[1, 1] = dt4 / 4.0 * q; Q[1, 4] = dt3 / 2.0 * q;
            Q[2, 2] = dt4 / 4.0 * q; Q[2, 5] = dt3 / 2.0 * q;
            Q[3, 0] = dt3 / 2.0 * q;
            Q[4, 1] = dt3 / 2.0 * q;
            Q[5, 2] = dt3 / 2.0 * q;
            Q[3, 3] = dt2 * q;
            Q[4, 4] = dt2 * q;
            Q[5, 5] = dt2 * q;
            return Q;
        }

        private Vector<double> MeasurementModel(Vector<double> x)
        {
            // Use the same measurement function as defined in H(x).
            return H(x);
        }

        private double NormalizeAngle(double a)
        {
            while (a > Math.PI)
                a -= 2.0 * Math.PI;
            while (a < -Math.PI)
                a += 2.0 * Math.PI;
            return a;
        }
        #endregion
    }
    #endregion

    #region IMMFilter Implementation
    /// <summary>
    /// Interacting Multiple Model (IMM) filter that runs both an EKF and a UKF
    /// </summary>
    public class IMMFilter : ITrackerFilter
    {
        private ITrackerFilter[] filters;    // filters[0] = EKF mode, filters[1] = UKF mode.
        private double[] modeProbabilities;  // current mode probabilities (sums to 1)
        private double[,] transitionMatrix;  // 2x2 model transition probability matrix
        private int M;                       // number of models (=2)

        public IMMFilter(ITrackerFilter ekf, ITrackerFilter ukf)
        {
            filters = new ITrackerFilter[2];
            filters[0] = ekf;
            filters[1] = ukf;
            M = 2;
            modeProbabilities = new double[2] { 0.5, 0.5 };
            // High probability to remain in the same mode.
            transitionMatrix = new double[2, 2];
            transitionMatrix[0, 0] = 0.97; transitionMatrix[0, 1] = 0.03;
            transitionMatrix[1, 0] = 0.03; transitionMatrix[1, 1] = 0.97;
        }

        public Vector<double> State
        {
            get { return CombinedState(); }
            set { /* Optionally, set state in each filter. */ }
        }
        public Matrix<double> Covariance
        {
            get { return CombinedCovariance(); }
            set { /* Optionally, set covariance in each filter. */ }
        }

        public void Predict(double dt)
        {
            // IMM mixing: mix the state and covariance for each mode.
            MixStates();
            foreach (var filter in filters)
            {
                filter.Predict(dt);
            }
        }

        public void Update(Vector<double> z)
        {
            // For IMM, we use the default measurement noise from the first mode.
            Update(z, filters[0].GetMeasurementNoiseCov());
        }

        /// <summary>
        /// IMM update overload that accepts a custom measurement covariance.
        /// </summary>
        public void Update(Vector<double> z, Matrix<double> customMeasurementCov)
        {
            double[] likelihoods = new double[M];
            // Run update on each mode and compute the likelihood.
            for (int j = 0; j < M; j++)
            {
                Vector<double> zPred = filters[j].H(filters[j].State);
                Vector<double> y = z - zPred;
                y[1] = NormalizeAngle(y[1]);
                y[2] = NormalizeAngle(y[2]);
                Matrix<double> S_mat = filters[j].S();
                double det = S_mat.Determinant();
                if (det < 1e-12) det = 1e-12;
                var sInv = S_mat.Inverse();
                double exponent = -0.5 * (y.ToRowMatrix() * sInv * y.ToColumnMatrix())[0, 0];
                double normFactor = 1.0 / (Math.Pow(2 * Math.PI, 1.5) * Math.Sqrt(det));
                likelihoods[j] = normFactor * Math.Exp(exponent);
                filters[j].Update(z, customMeasurementCov);
            }
            // Update mode probabilities.
            double[] updatedProb = new double[M];
            double c = 0.0;
            for (int j = 0; j < M; j++)
            {
                double sum = 0.0;
                for (int i = 0; i < M; i++)
                {
                    sum += transitionMatrix[i, j] * modeProbabilities[i];
                }
                updatedProb[j] = likelihoods[j] * sum;
                c += updatedProb[j];
            }
            if (c < 1e-12) c = 1e-12;
            for (int j = 0; j < M; j++)
            {
                modeProbabilities[j] = updatedProb[j] / c;
            }
        }

        public Vector<double> H(Vector<double> x)
        {
            // Both modes share the same measurement function.
            return filters[0].H(x);
        }

        public Matrix<double> S()
        {
            Matrix<double> combinedS = DenseMatrix.Create(3, 3, 0.0);
            for (int j = 0; j < M; j++)
            {
                combinedS += modeProbabilities[j] * filters[j].S();
            }
            return combinedS;
        }

        public Matrix<double> GetMeasurementNoiseCov()
        {
            return filters[0].GetMeasurementNoiseCov();
        }

        private Vector<double> CombinedState()
        {
            Vector<double> combined = DenseVector.Create(filters[0].State.Count, 0.0);
            for (int j = 0; j < M; j++)
            {
                combined += modeProbabilities[j] * filters[j].State;
            }
            return combined;
        }

        private Matrix<double> CombinedCovariance()
        {
            Vector<double> combined = CombinedState();
            Matrix<double> cov = DenseMatrix.Create(combined.Count, combined.Count, 0.0);
            for (int j = 0; j < M; j++)
            {
                Vector<double> diff = filters[j].State - combined;
                cov += modeProbabilities[j] * (filters[j].Covariance + diff.OuterProduct(diff));
            }
            return cov;
        }

        /// <summary>
        /// Mixing step: combine mode-conditioned states and covariances.
        /// </summary>
        private void MixStates()
        {
            double[,] mixingProb = new double[M, M];
            double[] c_j = new double[M];
            for (int j = 0; j < M; j++)
            {
                c_j[j] = 0.0;
                for (int i = 0; i < M; i++)
                {
                    c_j[j] += transitionMatrix[i, j] * modeProbabilities[i];
                }
            }
            for (int j = 0; j < M; j++)
            {
                for (int i = 0; i < M; i++)
                {
                    mixingProb[i, j] = (transitionMatrix[i, j] * modeProbabilities[i]) / (c_j[j] + 1e-12);
                }
            }
            // For each mode, compute the mixed state and covariance.
            for (int j = 0; j < M; j++)
            {
                Vector<double> mixedState = DenseVector.Create(filters[0].State.Count, 0.0);
                for (int i = 0; i < M; i++)
                {
                    mixedState += mixingProb[i, j] * filters[i].State;
                }
                Matrix<double> mixedCov = DenseMatrix.Create(filters[0].State.Count, filters[0].State.Count, 0.0);
                for (int i = 0; i < M; i++)
                {
                    Vector<double> diff = filters[i].State - mixedState;
                    mixedCov += mixingProb[i, j] * (filters[i].Covariance + diff.OuterProduct(diff));
                }
                filters[j].State = mixedState;
                filters[j].Covariance = mixedCov;
            }
        }

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI)
                angle -= 2 * Math.PI;
            while (angle < -Math.PI)
                angle += 2 * Math.PI;
            return angle;
        }
    }
    #endregion
}

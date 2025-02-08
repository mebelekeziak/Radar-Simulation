using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RealRadarSim.Tracking
{
    /// <summary>
    /// Improved Extended Kalman Filter with an iterative measurement update and
    /// less aggressive adaptive process noise.
    /// </summary>
    public class EKFFilter : ITrackerFilter
    {
        // Filter state: [px, py, pz, vx, vy, vz]
        public Vector<double> State { get; set; }
        public Matrix<double> Covariance { get; set; }

        // Nominal and adaptive acceleration noise parameters.
        private double sigmaA;
        private double adaptiveSigmaA;

        // Measurement noise covariance (3x3): [range, azimuth, elevation].
        private Matrix<double> R;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="initState">Initial state vector.</param>
        /// <param name="initCov">Initial state covariance.</param>
        /// <param name="accelNoise">Nominal acceleration noise.</param>
        public EKFFilter(Vector<double> initState, Matrix<double> initCov, double accelNoise)
        {
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaA = accelNoise;
            adaptiveSigmaA = sigmaA; // Start with nominal value.

            // Adjusted measurement noise
            R = DenseMatrix.OfArray(new double[,]
            {
                { 400 * 400,     0,              0 },
                { 0,         0.05 * 0.05,         0 },
                { 0,              0,        0.05 * 0.05 }
            });
        }

        /// <summary>
        /// Prediction step using a constant-velocity model and adaptive process noise.
        /// </summary>
        /// <param name="dt">Time step (seconds).</param>
        public void Predict(double dt)
        {
            // Constant-velocity state transition.
            var F = DenseMatrix.CreateIdentity(6);
            F[0, 3] = dt;
            F[1, 4] = dt;
            F[2, 5] = dt;
            State = F * State;

            // Process noise Q computed from adaptive acceleration noise.
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

            // Predict the covariance.
            Covariance = F * Covariance * F.Transpose() + Q;
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        /// <summary>
        /// Standard update that calls the custom update overload.
        /// </summary>
        /// <param name="z">Measurement vector.</param>
        public void Update(Vector<double> z)
        {
            Update(z, R);
        }

        /// <summary>
        /// Iterated EKF update using a custom measurement covariance.
        /// </summary>
        /// <param name="z">Measurement vector.</param>
        /// <param name="customMeasurementCov">Custom measurement noise covariance.</param>
        public void Update(Vector<double> z, Matrix<double> customMeasurementCov)
        {
            // --- Iterated Measurement Update ---
            int maxIterations = 3;
            double tolerance = 1e-3;
            Vector<double> statePrev = State.Clone();
            Matrix<double> K = null; // Kalman gain (from final iteration)

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // 1. Compute predicted measurement and innovation.
                Vector<double> hVal = H(State);
                Vector<double> y = z - hVal;
                y[1] = NormalizeAngle(y[1]);
                y[2] = NormalizeAngle(y[2]);

                // 2. Compute Jacobian at current state.
                Matrix<double> Hjac = Jacobian(State);

                // 3. Compute innovation covariance with added regularization.
                Matrix<double> S_mat = Hjac * Covariance * Hjac.Transpose() + customMeasurementCov;
                double reg = 1e-2; // Increased regularization for numerical stability.
                S_mat += DenseMatrix.CreateIdentity(S_mat.RowCount) * reg;

                // 4. Compute Kalman gain.
                var cholS = S_mat.Cholesky();
                Matrix<double> Sinv = cholS.Solve(DenseMatrix.CreateIdentity(S_mat.RowCount));
                K = Covariance * Hjac.Transpose() * Sinv;

                // 5. Update state.
                Vector<double> stateUpdate = K * y;
                State = State + stateUpdate;

                // 6. Check convergence.
                if ((State - statePrev).L2Norm() < tolerance)
                    break;
                statePrev = State.Clone();
            }

            // --- Adaptive Process Noise Update ---
            // Recompute final innovation for the updated state.
            Vector<double> finalHVal = H(State);
            Vector<double> finalY = z - finalHVal;
            finalY[1] = NormalizeAngle(finalY[1]);
            finalY[2] = NormalizeAngle(finalY[2]);

            Matrix<double> HjacFinal = Jacobian(State);
            Matrix<double> S_mat_final = HjacFinal * Covariance * HjacFinal.Transpose() + customMeasurementCov;
            double regFinal = 1e-2;
            S_mat_final += DenseMatrix.CreateIdentity(S_mat_final.RowCount) * regFinal;
            var cholFinal = S_mat_final.Cholesky();
            Matrix<double> SinvFinal = cholFinal.Solve(DenseMatrix.CreateIdentity(S_mat_final.RowCount));
            double md2 = (finalY.ToRowMatrix() * SinvFinal * finalY.ToColumnMatrix())[0, 0];

            // Use milder multipliers to update the adaptive noise.
            if (md2 > 9.0)
                adaptiveSigmaA = Math.Min(adaptiveSigmaA * 1.02, sigmaA * 5.0);
            else if (md2 < 3.0)
                adaptiveSigmaA = Math.Max(adaptiveSigmaA * 0.98, sigmaA);

            // --- Covariance Update ---
            var I = DenseMatrix.CreateIdentity(6);
            Matrix<double> KH = K * HjacFinal;
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K * customMeasurementCov * K.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());

            // Enforce a minimum variance on the diagonal.
            double minVar = 1e-2;
            for (int i = 0; i < Covariance.RowCount; i++)
            {
                if (Covariance[i, i] < minVar)
                    Covariance[i, i] = minVar;
            }
        }

        /// <summary>
        /// Nonlinear measurement function mapping state to measurement space.
        /// Returns [range, azimuth, elevation].
        /// </summary>
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

        /// <summary>
        /// Returns the innovation covariance S = H * Covariance * Hᵀ + R.
        /// </summary>
        public Matrix<double> S()
        {
            Matrix<double> Hjac = Jacobian(State);
            return Hjac * Covariance * Hjac.Transpose() + R;
        }

        /// <summary>
        /// Returns the filter’s intrinsic measurement noise covariance.
        /// </summary>
        public Matrix<double> GetMeasurementNoiseCov()
        {
            return R;
        }

        /// <summary>
        /// Computes the Jacobian of the measurement function at the current state.
        /// </summary>
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
            H[0, 0] = px / range;
            H[0, 1] = py / range;
            H[0, 2] = pz / range;

            // Partial derivatives for azimuth.
            H[1, 0] = -py / denomXY;
            H[1, 1] = px / denomXY;

            // Partial derivatives for elevation.
            double range2 = range * range;
            H[2, 0] = -pz * px / (range2 * rho);
            H[2, 1] = -pz * py / (range2 * rho);
            H[2, 2] = rho / range2;

            return H;
        }

        /// <summary>
        /// Normalizes an angle to the range [-π, π].
        /// </summary>
        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI)
                angle -= 2.0 * Math.PI;
            while (angle < -Math.PI)
                angle += 2.0 * Math.PI;
            return angle;
        }
    }
}

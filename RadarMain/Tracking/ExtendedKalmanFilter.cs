using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RealRadarSim.Utils;

namespace RealRadarSim.Tracking
{
    /// <summary>
    /// Extended Kalman Filter tuned for maneuvering targets using a constant–acceleration model.
    /// The state vector is [px, py, pz, vx, vy, vz, ax, ay, az].
    /// </summary>
    public class ManeuveringEKF : ITrackerFilter
    {
        // State vector: [px, py, pz, vx, vy, vz, ax, ay, az]
        public Vector<double> State { get; set; }
        public Matrix<double> Covariance { get; set; }

        // Jerk noise parameter (used to drive the acceleration model) and its adaptive version.
        private double sigmaJ;
        private double adaptiveSigmaJ;

        // Measurement noise covariance (3x3): [range, azimuth, elevation].
        private Matrix<double> R;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="initState">Initial state vector (dimension 9).</param>
        /// <param name="initCov">Initial state covariance (9x9 matrix).</param>
        /// <param name="jerkNoise">Nominal jerk noise (standard deviation).</param>
        public ManeuveringEKF(Vector<double> initState, Matrix<double> initCov, double jerkNoise)
        {
            if (initState.Count != 9)
                throw new ArgumentException("Initial state must be 9-dimensional for a constant-acceleration model.");
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaJ = jerkNoise;
            adaptiveSigmaJ = sigmaJ; // Start with nominal value.

            // Adjusted measurement noise covariance (range in meters, angles in radians).
            R = DenseMatrix.OfArray(new double[,]
            {
                { 400 * 400,       0,              0 },
                { 0,          0.1 * 0.1,         0 },
                { 0,               0,       0.1 * 0.1 }
            });
        }

        /// <summary>
        /// Prediction step using a constant–acceleration model and adaptive jerk (process) noise.
        /// </summary>
        /// <param name="dt">Time step (seconds).</param>
        public void Predict(double dt)
        {
            // Build the 9x9 state transition matrix F for a constant-acceleration model.
            // For each axis (x, y, z):
            //   p = p + dt*v + 0.5*dt^2*a
            //   v = v + dt*a
            //   a = a (assumed constant over dt)
            var F = DenseMatrix.CreateIdentity(9);
            for (int d = 0; d < 3; d++)
            {
                // Indices: position d, velocity d+3, acceleration d+6.
                F[d, d + 3] = dt;
                F[d, d + 6] = 0.5 * dt * dt;
                F[d + 3, d + 6] = dt;
            }
            State = F * State;

            // Process noise covariance Q based on a white-noise jerk model.
            double q = adaptiveSigmaJ * adaptiveSigmaJ;
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt3 * dt;
            double dt5 = dt4 * dt;

            // Q will be a 9x9 matrix where, for each dimension d,
            //   Q[position, position] = dt^5/20 * q
            //   Q[position, velocity] = dt^4/8 * q, etc.
            var Q = DenseMatrix.Create(9, 9, 0.0);
            for (int d = 0; d < 3; d++)
            {
                int pos = d;      // p_x, p_y, or p_z
                int vel = d + 3;  // v_x, v_y, or v_z
                int acc = d + 6;  // a_x, a_y, or a_z

                Q[pos, pos] = dt5 / 20.0 * q;
                Q[pos, vel] = dt4 / 8.0 * q;
                Q[pos, acc] = dt3 / 6.0 * q;
                Q[vel, pos] = dt4 / 8.0 * q;
                Q[vel, vel] = dt3 / 3.0 * q;
                Q[vel, acc] = dt2 / 2.0 * q;
                Q[acc, pos] = dt3 / 6.0 * q;
                Q[acc, vel] = dt2 / 2.0 * q;
                Q[acc, acc] = dt * q;   // was 'dt' — now 'dt * q'
            }

            // Predict the covariance.
            Covariance = F * Covariance * F.Transpose() + Q;
            Covariance = 0.5 * (Covariance + Covariance.Transpose());
        }

        /// <summary>
        /// Standard update that calls the custom update overload.
        /// </summary>
        /// <param name="z">Measurement vector [range, azimuth, elevation].</param>
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
            Matrix<double> K = null; // Kalman gain from final iteration.

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // 1. Compute predicted measurement and innovation.
                Vector<double> hVal = H(State);
                Vector<double> y = z - hVal;
                y[1] = MathUtil.NormalizeAngle(y[1]);
                y[2] = MathUtil.NormalizeAngle(y[2]);

                // 2. Compute Jacobian at current state.
                Matrix<double> Hjac = Jacobian(State);

                // 3. Compute innovation covariance with added regularization.
                Matrix<double> S_mat = Hjac * Covariance * Hjac.Transpose() + customMeasurementCov;
                double reg = 1e-2; // Regularization for numerical stability.
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
            // Recompute final innovation.
            Vector<double> finalHVal = H(State);
            Vector<double> finalY = z - finalHVal;
            finalY[1] = MathUtil.NormalizeAngle(finalY[1]);
            finalY[2] = MathUtil.NormalizeAngle(finalY[2]);

            Matrix<double> HjacFinal = Jacobian(State);
            Matrix<double> S_mat_final = HjacFinal * Covariance * HjacFinal.Transpose() + customMeasurementCov;
            double regFinal = 1e-2;
            S_mat_final += DenseMatrix.CreateIdentity(S_mat_final.RowCount) * regFinal;
            var cholFinal = S_mat_final.Cholesky();
            Matrix<double> SinvFinal = cholFinal.Solve(DenseMatrix.CreateIdentity(S_mat_final.RowCount));
            double md2 = (finalY.ToRowMatrix() * SinvFinal * finalY.ToColumnMatrix())[0, 0];

            // More aggressive adaptation for maneuvering targets.
            if (md2 > 9.0)
                adaptiveSigmaJ = Math.Min(adaptiveSigmaJ * 1.2, sigmaJ * 10.0);
            else if (md2 < 3.0)
                adaptiveSigmaJ = Math.Max(adaptiveSigmaJ * 0.8, sigmaJ);

            // --- Covariance Update ---
            var I = DenseMatrix.CreateIdentity(9);
            Matrix<double> KH = K * HjacFinal;
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K * customMeasurementCov * K.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());

            // Enforce a minimum variance on the diagonal.
            double minVar = 400;
            for (int i = 0; i < Covariance.RowCount; i++)
            {
                if (Covariance[i, i] < minVar)
                    Covariance[i, i] = minVar;
            }
        }

        /// <summary>
        /// Nonlinear measurement function mapping state to measurement space.
        /// Returns [range, azimuth, elevation] using position only.
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
        /// Computes the Jacobian of the measurement function at the current state.
        /// Returns a 3x9 matrix. Only the position components (columns 0-2) are nonzero.
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

            var H = DenseMatrix.Create(3, 9, 0.0);

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

            // The remaining columns (velocity and acceleration) are zero.
            return H;
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

    }
}

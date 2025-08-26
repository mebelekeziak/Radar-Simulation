using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
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

        // ---------------------------------------------------------------------
        //  Internal helpers for robust linear algebra (SPD solve with fallback)
        // ---------------------------------------------------------------------
        private static Matrix<double> SolveSymmetric(Matrix<double> A, Matrix<double> B)
        {
            try
            {
                var chol = A.Cholesky();
                return chol.Solve(B);
            }
            catch
            {
                var pinv = PseudoInverse(A);
                return pinv * B;
            }
        }

        private static Vector<double> SolveSymmetric(Matrix<double> A, Vector<double> b)
        {
            try
            {
                var chol = A.Cholesky();
                return chol.Solve(b);
            }
            catch
            {
                var pinv = PseudoInverse(A);
                return pinv * b;
            }
        }

        private static Matrix<double> PseudoInverse(Matrix<double> A, double tol = 1e-10)
        {
            var svd = A.Svd(true);
            var s = svd.S;
            int r = s.Count;
            var invS = DenseMatrix.CreateDiagonal(r, r, i => s[i] > tol ? 1.0 / s[i] : 0.0);
            // A = U * S * V^T → A^+ = V * S^+ * U^T
            return svd.VT.Transpose() * invS * svd.U.Transpose();
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
            S_mat = S_mat + DenseMatrix.CreateIdentity(S_mat.RowCount) * reg;

            // 4. Compute Kalman gain without forming S^{-1} explicitly (robust solve)
            Matrix<double> PHt = Covariance * Hjac.Transpose();
            Matrix<double> Xt = SolveSymmetric(S_mat, PHt.Transpose()); // (m x n)
            K = Xt.Transpose();                                         // (n x m)

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
            S_mat_final = S_mat_final + DenseMatrix.CreateIdentity(S_mat_final.RowCount) * regFinal;
            // Solve S x = y, then y^T x (robust)
            var x_md = SolveSymmetric(S_mat_final, finalY);
            double md2 = finalY.DotProduct(x_md);

            // More aggressive adaptation for maneuvering targets.
            if (md2 > 9.0)
                adaptiveSigmaJ = Math.Min(adaptiveSigmaJ * 1.2, sigmaJ * 10.0);
            else if (md2 < 3.0)
                adaptiveSigmaJ = Math.Max(adaptiveSigmaJ * 0.8, sigmaJ);

            // --- Covariance Update ---
            var I = DenseMatrix.CreateIdentity(9);
            // Recompute gain at the final linearization (robust solve)
            Matrix<double> PHt_final = Covariance * HjacFinal.Transpose();
            Matrix<double> Xt_final = SolveSymmetric(S_mat_final, PHt_final.Transpose());
            Matrix<double> K_final = Xt_final.Transpose();
            Matrix<double> KH = K_final * HjacFinal;
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K_final * customMeasurementCov * K_final.Transpose();
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
        /// Extended measurement function with Doppler: [range, azimuth, elevation, range_rate].
        /// Range rate is the radial velocity projected along the line of sight.
        /// </summary>
        public Vector<double> HWithDoppler(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double vx = x[3], vy = x[4], vz = x[5];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double az = Math.Atan2(py, px);
            double rho = Math.Sqrt(px * px + py * py);
            if (rho < 1e-9) rho = 1e-9;
            double el = Math.Atan2(pz, rho);
            double dot = px * vx + py * vy + pz * vz;
            double rr = dot / range;
            return DenseVector.OfArray(new double[] { range, az, el, rr });
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
        /// Jacobian for the extended measurement [range, azimuth, elevation, range_rate].
        /// </summary>
        private Matrix<double> JacobianWithDoppler(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double vx = x[3], vy = x[4], vz = x[5];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double denomXY = px * px + py * py;
            if (denomXY < 1e-9) denomXY = 1e-9;
            double rho = Math.Sqrt(denomXY);
            if (rho < 1e-9) rho = 1e-9;

            var H = DenseMatrix.Create(4, 9, 0.0);

            // range
            H[0, 0] = px / range;
            H[0, 1] = py / range;
            H[0, 2] = pz / range;

            // azimuth
            H[1, 0] = -py / denomXY;
            H[1, 1] = px / denomXY;

            // elevation
            double range2 = range * range;
            H[2, 0] = -pz * px / (range2 * rho);
            H[2, 1] = -pz * py / (range2 * rho);
            H[2, 2] = rho / range2;

            // range_rate
            double dot = px * vx + py * vy + pz * vz;
            double r3 = range2 * range;
            H[3, 0] = (vx / range) - (dot * px) / r3;  // d(rr)/dpx
            H[3, 1] = (vy / range) - (dot * py) / r3;  // d(rr)/dpy
            H[3, 2] = (vz / range) - (dot * pz) / r3;  // d(rr)/dpz
            H[3, 3] = px / range;                      // d(rr)/dvx
            H[3, 4] = py / range;                      // d(rr)/dvy
            H[3, 5] = pz / range;                      // d(rr)/dvz

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

        /// <summary>
        /// Optional update that supports a 4D measurement vector with Doppler (range rate).
        /// If z has 3 elements, falls back to the standard position-only model.
        /// </summary>
        public void UpdateWithDoppler(Vector<double> z, Matrix<double> customMeasurementCov)
        {
            if (z.Count == 3)
            {
                Update(z, customMeasurementCov);
                return;
            }
            if (z.Count != 4)
                throw new ArgumentException("Measurement vector must be length 3 or 4.");

            int maxIterations = 3;
            double tolerance = 1e-3;
            Vector<double> statePrev = State.Clone();

            for (int iter = 0; iter < maxIterations; iter++)
            {
                var hVal = HWithDoppler(State);
                var y = z - hVal;
                y[1] = MathUtil.NormalizeAngle(y[1]);
                y[2] = MathUtil.NormalizeAngle(y[2]);

                var Hjac = JacobianWithDoppler(State);
                var S_mat = Hjac * Covariance * Hjac.Transpose() + customMeasurementCov +
                            DenseMatrix.CreateIdentity(4) * 1e-2;

                var PHt = Covariance * Hjac.Transpose();
                var Xt = SolveSymmetric(S_mat, PHt.Transpose());
                var K = Xt.Transpose();

                var stateUpdate = K * y;
                State = State + stateUpdate;

                if ((State - statePrev).L2Norm() < tolerance)
                    break;
                statePrev = State.Clone();
            }

            // Adaptive process noise based on final innovation
            var finalH = HWithDoppler(State);
            var finalY = z - finalH;
            finalY[1] = MathUtil.NormalizeAngle(finalY[1]);
            finalY[2] = MathUtil.NormalizeAngle(finalY[2]);

            var Hfinal = JacobianWithDoppler(State);
            var Sfinal = Hfinal * Covariance * Hfinal.Transpose() + customMeasurementCov +
                         DenseMatrix.CreateIdentity(4) * 1e-2;
            var x_md = SolveSymmetric(Sfinal, finalY);
            double md2 = finalY.DotProduct(x_md);

            if (md2 > 12.0)
                adaptiveSigmaJ = Math.Min(adaptiveSigmaJ * 1.2, sigmaJ * 10.0);
            else if (md2 < 4.0)
                adaptiveSigmaJ = Math.Max(adaptiveSigmaJ * 0.8, sigmaJ);

            // Covariance update (Joseph form)
            var I4 = DenseMatrix.CreateIdentity(9);
            var PHt_f = Covariance * Hfinal.Transpose();
            var Xt_f = SolveSymmetric(Sfinal, PHt_f.Transpose());
            var Kf = Xt_f.Transpose();
            var KH = Kf * Hfinal;
            Covariance = (I4 - KH) * Covariance * (I4 - KH).Transpose() + Kf * customMeasurementCov * Kf.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());

            // Minimum variance guard on the diagonal
            double minVar = 400;
            for (int i = 0; i < Covariance.RowCount; i++)
                if (Covariance[i, i] < minVar) Covariance[i, i] = minVar;
        }

    }
}

using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RealRadarSim.Tracking
{
    public class AdvancedEKF
    {
        public Vector<double> State;       // [px, py, pz, vx, vy, vz]
        public Matrix<double> Covariance;  // 6x6
        private double sigmaA;             // Acceleration noise
        private Matrix<double> R;          // Measurement noise covariance (3x3)

        public AdvancedEKF(Vector<double> initState, Matrix<double> initCov, double accelNoise)
        {
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaA = accelNoise;

            // Measurement noise for [range, azimuth, elevation]
            // R[0,0] = (range std dev)^2
            // R[1,1] = (az std dev)^2
            // R[2,2] = (el std dev)^2
            // Example: range = 50, az/el = 0.001
            R = DenseMatrix.OfArray(new double[,]
            {
                { 50 * 50, 0,       0       },
                { 0,       0.001*0.001, 0   },
                { 0,       0,       0.001*0.001 }
            });
        }

        /// <summary>
        /// Predict the next state using a constant-velocity model with process noise.
        /// </summary>
        /// <param name="dt">Time step in seconds</param>
        public void Predict(double dt)
        {
            // State transition matrix F
            var F = DenseMatrix.CreateIdentity(6);
            F[0, 3] = dt;  // px += vx * dt
            F[1, 4] = dt;  // py += vy * dt
            F[2, 5] = dt;  // pz += vz * dt

            // Predict state
            State = F * State;

            // Process noise matrix Q (for constant velocity with random accel)
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt3 * dt;
            double q = sigmaA * sigmaA;

            // Construct Q
            var Q = DenseMatrix.Create(6, 6, 0);
            // Position part
            Q[0, 0] = dt4 / 4.0 * q; Q[0, 3] = dt3 / 2.0 * q;
            Q[1, 1] = dt4 / 4.0 * q; Q[1, 4] = dt3 / 2.0 * q;
            Q[2, 2] = dt4 / 4.0 * q; Q[2, 5] = dt3 / 2.0 * q;

            // Cross terms
            Q[3, 0] = dt3 / 2.0 * q; Q[4, 1] = dt3 / 2.0 * q; Q[5, 2] = dt3 / 2.0 * q;

            // Velocity part
            Q[3, 3] = dt2 * q;
            Q[4, 4] = dt2 * q;
            Q[5, 5] = dt2 * q;

            // Predict covariance
            Covariance = F * Covariance * F.Transpose() + Q;
        }

        /// <summary>
        /// Update step with a measurement vector z = [range, az, el].
        /// Uses the Joseph form for covariance update to improve numerical stability.
        /// </summary>
        /// <param name="z">Measurement: [range, azimuth, elevation]</param>
        public void Update(Vector<double> z)
        {
            // 1) Compute measurement prediction h(x)
            var hVal = H(State);

            // 2) Measurement residual y = z - h(x)
            var y = z - hVal;
            // Normalize angle residuals
            y[1] = NormalizeAngle(y[1]); // az
            y[2] = NormalizeAngle(y[2]); // el

            // 3) Compute Jacobian of h at current state
            var Hjac = Jacobian(State);

            // 4) Compute residual covariance S = H * P * H^T + R
            var S = Hjac * Covariance * Hjac.Transpose() + R;

            // 5) Kalman gain K = P * H^T * S^-1
            // For better numerical stability, consider using S.Cholesky().Solve(...) if S is well-conditioned:
            //   var Sinv = S.Cholesky().Solve(DenseMatrix.CreateIdentity(3));
            //   var K    = Covariance * Hjac.Transpose() * Sinv;
            var K = Covariance * Hjac.Transpose() * S.Inverse();

            // 6) State update
            State = State + K * y;

            // 7) Covariance update using the Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
            var I = DenseMatrix.CreateIdentity(6);
            var KH = K * Hjac;
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K * R * K.Transpose();

            // Optionally, you can do an outlier check before applying the update:
            // double mahaDist = y.ToRowMatrix() * S.Inverse() * y.ToColumnMatrix();
            // if(mahaDist < threshold) { apply update } else { skip update };
        }

        /// <summary>
        /// (Optional) Returns the current measurement covariance S for debugging or gating checks.
        /// </summary>
        public Matrix<double> S()
        {
            var Hjac = Jacobian(State);
            return Hjac * Covariance * Hjac.Transpose() + R;
        }

        /// <summary>
        /// Measurement function h(x) = [range, az, el].
        /// range = sqrt(px^2 + py^2 + pz^2)
        /// az = atan2(py, px)
        /// el = atan2(pz, sqrt(px^2 + py^2))
        /// </summary>
        public Vector<double> H(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];

            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;  // avoid division by zero

            double az = Math.Atan2(py, px);

            double rho = Math.Sqrt(px * px + py * py);
            if (rho < 1e-9) rho = 1e-9;
            double el = Math.Atan2(pz, rho);

            return DenseVector.OfArray(new double[] { range, az, el });
        }

        /// <summary>
        /// Jacobian of h(x) with respect to x = [px, py, pz, vx, vy, vz].
        /// For your current measurement, partial derivatives wrt velocity are zero.
        /// Hence only the top-left 3x3 portion is non-zero.
        /// </summary>
        public Matrix<double> Jacobian(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];

            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6; // avoid division by zero

            double denomXY = px * px + py * py;
            if (denomXY < 1e-9) denomXY = 1e-9;

            double rho = Math.Sqrt(denomXY);
            if (rho < 1e-9) rho = 1e-9;

            var H = DenseMatrix.Create(3, 6, 0.0);

            // partial(range)/partial(px,py,pz)
            H[0, 0] = px / range;
            H[0, 1] = py / range;
            H[0, 2] = pz / range;

            // partial(az)/partial(px,py)
            // az = atan2(py, px)
            // d(atan2(y, x))/dx = -y / (x^2 + y^2)
            // d(atan2(y, x))/dy =  x / (x^2 + y^2)
            H[1, 0] = -py / denomXY;
            H[1, 1] = px / denomXY;

            // partial(el)/partial(px,py,pz)
            // el = atan2(pz, sqrt(px^2 + py^2))
            double range2 = range * range;
            H[2, 0] = -pz * px / (range2 * rho);
            H[2, 1] = -pz * py / (range2 * rho);
            H[2, 2] = rho / range2;

            // partial(...) wrt vx, vy, vz are zero for these measurements
            // (If you measure radial velocity, you'd add those terms here.)

            return H;
        }

        /// <summary>
        /// Normalize an angle to [-pi, pi].
        /// </summary>
        private double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2.0 * Math.PI;
            while (a < -Math.PI) a += 2.0 * Math.PI;
            return a;
        }
    }
}

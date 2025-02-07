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

            // Updated measurement noise:
            // - Range std = 50  --> variance = 2500
            // - Azimuth std = 0.1  --> variance = 0.1^2 = 0.01
            // - Elevation std = 0.01  --> variance = 0.01^2 = 0.0001
            R = DenseMatrix.OfArray(new double[,]
            {
                { 50 * 50,     0,             0 },
                { 0,       0.1 * 0.1,         0 },
                { 0,             0,       0.01 * 0.01 }
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
            // Compute measurement prediction and innovation.
            Vector<double> hVal = H(State);
            Vector<double> y = z - hVal;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);

            // Compute Jacobian of the measurement function.
            Matrix<double> Hjac = Jacobian(State);

            // Compute innovation covariance S and add regularization.
            Matrix<double> S_mat = Hjac * Covariance * Hjac.Transpose() + customMeasurementCov;
            double reg = 1e-3;  // Increase this value if needed.
            S_mat += DenseMatrix.CreateIdentity(S_mat.RowCount) * reg;

            // Compute inverse via Cholesky.
            var cholS = S_mat.Cholesky();
            Matrix<double> Sinv = cholS.Solve(DenseMatrix.CreateIdentity(3));

            // --- Adaptive Process Noise Update ---
            double md2 = (y.ToRowMatrix() * Sinv * y.ToColumnMatrix())[0, 0];
            if (md2 > 9.0)
                adaptiveSigmaA = Math.Min(adaptiveSigmaA * 1.05, sigmaA * 5.0);
            else if (md2 < 3.0)
                adaptiveSigmaA = Math.Max(adaptiveSigmaA * 0.95, sigmaA);

            // Compute Kalman gain and update state.
            Matrix<double> K = Covariance * Hjac.Transpose() * Sinv;
            State = State + K * y;

            // Update covariance.
            var I = DenseMatrix.CreateIdentity(6);
            Matrix<double> KH = K * Hjac;
            Covariance = (I - KH) * Covariance * (I - KH).Transpose() + K * customMeasurementCov * K.Transpose();
            Covariance = 0.5 * (Covariance + Covariance.Transpose());

            // --- Enforce a Covariance Floor ---
            double minVar = 1e-2;  // Minimum allowed variance on each diagonal element.
            for (int i = 0; i < Covariance.RowCount; i++)
            {
                if (Covariance[i, i] < minVar)
                    Covariance[i, i] = minVar;
            }
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
}

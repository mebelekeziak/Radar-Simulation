using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RealRadarSim.Tracking
{
    public class AdvancedEKF
    {
        public Vector<double> State;
        public Matrix<double> Covariance;
        private double sigmaA;
        private Matrix<double> R;

        public AdvancedEKF(Vector<double> initState, Matrix<double> initCov, double accelNoise)
        {
            State = initState.Clone();
            Covariance = initCov.Clone();
            sigmaA = accelNoise;
            R = DenseMatrix.OfArray(new double[,]
            {
                { 50 * 50,       0,             0 },
                { 0,       0.001 * 0.001,         0 },
                { 0,              0,       0.001 * 0.001 }
            });
        }

        public void Predict(double dt)
        {
            var F = DenseMatrix.CreateIdentity(6);
            F[0, 3] = dt;
            F[1, 4] = dt;
            F[2, 5] = dt;
            State = F * State;
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt3 * dt;
            double q = sigmaA * sigmaA;
            var Q = DenseMatrix.Create(6, 6, 0);
            Q[0, 0] = dt4 / 4 * q; Q[0, 3] = dt3 / 2 * q;
            Q[3, 0] = dt3 / 2 * q; Q[3, 3] = dt2 * q;
            Q[1, 1] = dt4 / 4 * q; Q[1, 4] = dt3 / 2 * q;
            Q[4, 1] = dt3 / 2 * q; Q[4, 4] = dt2 * q;
            Q[2, 2] = dt4 / 4 * q; Q[2, 5] = dt3 / 2 * q;
            Q[5, 2] = dt3 / 2 * q; Q[5, 5] = dt2 * q;
            Covariance = F * Covariance * F.Transpose() + Q;
        }

        public void Update(Vector<double> z)
        {
            var hVal = H(State);
            var y = z - hVal;
            y[1] = NormalizeAngle(y[1]);
            y[2] = NormalizeAngle(y[2]);
            var Hjac = Jacobian(State);
            var S = Hjac * Covariance * Hjac.Transpose() + R;
            var K = Covariance * Hjac.Transpose() * S.Inverse();
            State = State + K * y;
            var I = DenseMatrix.CreateIdentity(6);
            Covariance = (I - K * Hjac) * Covariance;
        }

        public Matrix<double> S()
        {
            var Hjac = Jacobian(State);
            return Hjac * Covariance * Hjac.Transpose() + R;
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

        public Matrix<double> Jacobian(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double range = Math.Sqrt(px * px + py * py + pz * pz);
            if (range < 1e-6) range = 1e-6;
            double denomXY = px * px + py * py;
            if (denomXY < 1e-9) denomXY = 1e-9;
            double rho = Math.Sqrt(denomXY);
            if (rho < 1e-9) rho = 1e-9;
            var H = DenseMatrix.Create(3, 6, 0.0);
            H[0, 0] = px / range;
            H[0, 1] = py / range;
            H[0, 2] = pz / range;
            H[1, 0] = -py / denomXY;
            H[1, 1] = px / denomXY;
            double range2 = range * range;
            H[2, 0] = -pz * px / (range2 * rho);
            H[2, 1] = -pz * py / (range2 * rho);
            H[2, 2] = rho / range2;
            return H;
        }

        private double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2.0 * Math.PI;
            while (a < -Math.PI) a += 2.0 * Math.PI;
            return a;
        }
    }
}

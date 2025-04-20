using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Distributions;

namespace RealRadarSim.Tracking
{
    public class ManeuveringEKF : ITrackerFilter
    {
        private const int StateDim = 9;
        private readonly int MeasDim;
        private const double AngleEps = 1e-9;
        private const double RangeEps = 1e-6;

        // tuning parameters
        private readonly double sigmaJ0;
        private double sigmaJ;
        private readonly double velNoiseStd;
        private readonly double gateAlpha;

        // public state & covariance 
        public Vector<double> State { get; set; }
        public Matrix<double> Covariance { get; set; }

        // measurement noise covariance
        private readonly Matrix<double> R;

        // scratch buffers
        private readonly Matrix<double> F = DenseMatrix.CreateIdentity(StateDim);
        private readonly Matrix<double> Q = new DenseMatrix(StateDim, StateDim);
        private readonly Matrix<double> Hbuf;                      
        private readonly Matrix<double> Sbuf;
        private readonly Vector<double> y;
        private readonly Matrix<double> I = DenseMatrix.CreateIdentity(StateDim);

        private readonly double[,] Qbar = new double[StateDim, StateDim];

        public ManeuveringEKF(
            Vector<double> x0,
            Matrix<double> P0,
            double jerkStd,
            bool includeDoppler = false,
            double velocityNoiseStd = 1.0,
            double gateProbabilityAlpha = 0.05,
            double rangeNoiseStd = 40.0,
            double angleNoiseStdRad = 0.2)
        {
            if (x0.Count != StateDim) throw new ArgumentException($"x0 must be {StateDim}-vector");
            if (P0.RowCount != StateDim || P0.ColumnCount != StateDim) throw new ArgumentException($"P0 must be {StateDim}×{StateDim}");

            State = x0.Clone();
            Covariance = Symmetrize(P0.Clone());

            sigmaJ0 = jerkStd;
            sigmaJ = sigmaJ0;
            velNoiseStd = velocityNoiseStd;
            gateAlpha = Math.Clamp(gateProbabilityAlpha, 1e-6, 0.25);

            MeasDim = includeDoppler ? 4 : 3;

            R = DenseMatrix.Create(MeasDim, MeasDim, 0.0);
            R[0, 0] = rangeNoiseStd * rangeNoiseStd;
            R[1, 1] = angleNoiseStdRad * angleNoiseStdRad;
            R[2, 2] = angleNoiseStdRad * angleNoiseStdRad;
            if (includeDoppler) R[3, 3] = velNoiseStd * velNoiseStd;

            Hbuf = new DenseMatrix(MeasDim, StateDim);
            Sbuf = new DenseMatrix(MeasDim, MeasDim);
            y = new DenseVector(MeasDim);

            PrecomputeQbar();
        }

        // ---------------- prediction ----------------
        public void Predict(double dt)
        {
            BuildF(dt);
            ScaleQ(dt);

            Covariance = F * Covariance * F.Transpose() + Q;
            Covariance = Symmetrize(Covariance);
            State = F * State;
        }

        // ---------------- update ----------------
        public void Update(Vector<double> z) => Update(z, R);

        public void Update(Vector<double> z, Matrix<double> Rcustom)
        {
            if (z.Count != MeasDim) throw new ArgumentException($"z must be {MeasDim}-vector");

            // χ²-gate
            if (gateAlpha > 0.0)
            {
                double gate = ChiSquared.InvCDF(MeasDim, 1.0 - gateAlpha);
                if (ComputeNIS(z, Rcustom) > gate) return;
            }

            const int MaxIter = 3; const double Tol = 1e-3;
            Vector<double> xPrev = State.Clone();
            Matrix<double> K = null;

            for (int i = 0; i < MaxIter; ++i)
            {
                var h = MeasurementFunction(State);
                y.SetSubVector(0, MeasDim, z - h);
                NormalizeInnovationAngles(y);

                ComputeJacobian(State, Hbuf);

                Sbuf.SetSubMatrix(0, 0, Hbuf * Covariance * Hbuf.Transpose());
                Sbuf.Add(Rcustom, Sbuf);

                var Sinv = Sbuf.Cholesky().Solve(DenseMatrix.CreateIdentity(MeasDim));
                K = Covariance * Hbuf.Transpose() * Sinv;

                State += K * y;
                if ((State - xPrev).L2Norm() < Tol) break;
                xPrev = State.Clone();
            }

            var IKH = I - K * Hbuf;
            Covariance = IKH * Covariance * IKH.Transpose() + K * Rcustom * K.Transpose();
            Covariance = Symmetrize(Covariance);

            AdaptJerk(z);
        }

        // ------------- ITrackerFilter helpers -------------
        public Vector<double> H(Vector<double> x) => MeasurementFunction(x);

        public Matrix<double> S()
        {
            ComputeJacobian(State, Hbuf);
            return Hbuf * Covariance * Hbuf.Transpose() + R;
        }

        public Matrix<double> GetMeasurementNoiseCov() => R.Clone();

        // ------------- internal maths ---------------------
        private Vector<double> MeasurementFunction(Vector<double> x)
        {
            double px = x[0], py = x[1], pz = x[2];
            double vx = x[3], vy = x[4], vz = x[5];

            double r = Math.Sqrt(px * px + py * py + pz * pz); r = Math.Max(r, RangeEps);
            double az = Math.Atan2(py, px);
            double rho = Math.Sqrt(px * px + py * py); rho = Math.Max(rho, AngleEps);
            double el = Math.Atan2(pz, rho);

            if (MeasDim == 3) return DenseVector.OfArray(new[] { r, az, el });

            double vr = (px * vx + py * vy + pz * vz) / r;
            return DenseVector.OfArray(new[] { r, az, el, vr });
        }

        private void ComputeJacobian(Vector<double> x, Matrix<double> J)
        {
            J.Clear();

            double px = x[0], py = x[1], pz = x[2];
            double vx = x[3], vy = x[4], vz = x[5];

            double r2 = Math.Max(px * px + py * py + pz * pz, RangeEps);
            double r = Math.Sqrt(r2);
            double d2 = Math.Max(px * px + py * py, AngleEps);
            double rho = Math.Sqrt(d2);

            J[0, 0] = px / r; J[0, 1] = py / r; J[0, 2] = pz / r;          // range
            J[1, 0] = -py / d2; J[1, 1] = px / d2;                    // azimuth
            J[2, 0] = -pz * px / (r2 * rho); J[2, 1] = -pz * py / (r2 * rho); J[2, 2] = rho / r2; // elevation

            if (MeasDim == 3) return;

            double vr = (px * vx + py * vy + pz * vz) / r;
            J[3, 0] = (vx * r - vr * px) / r2;
            J[3, 1] = (vy * r - vr * py) / r2;
            J[3, 2] = (vz * r - vr * pz) / r2;
            J[3, 3] = px / r; J[3, 4] = py / r; J[3, 5] = pz / r;          // Doppler
        }

        private void BuildF(double dt)
        {
            F.Clear();
            for (int i = 0; i < StateDim; ++i) F[i, i] = 1.0;

            double dt2 = 0.5 * dt * dt;
            for (int a = 0; a < 3; ++a)
            {
                int p = a, v = a + 3, acc = a + 6;
                F[p, v] = dt; F[p, acc] = dt2; F[v, acc] = dt;
            }
        }

        private void PrecomputeQbar()
        {
            for (int a = 0; a < 3; ++a)
            {
                int p = a, v = a + 3, acc = a + 6;
                Qbar[p, p] = 1.0 / 20.0;
                Qbar[p, v] = Qbar[v, p] = 1.0 / 8.0;
                Qbar[p, acc] = Qbar[acc, p] = 1.0 / 6.0;
                Qbar[v, v] = 1.0 / 3.0;
                Qbar[v, acc] = Qbar[acc, v] = 1.0 / 2.0;
                Qbar[acc, acc] = 1.0;
            }
        }

        private void ScaleQ(double dt)
        {
            double dt5 = Math.Pow(dt, 5), dt4 = Math.Pow(dt, 4), dt3 = Math.Pow(dt, 3), dt2 = Math.Pow(dt, 2);
            Q.Clear();
            for (int a = 0; a < 3; ++a)
            {
                int p = a, v = a + 3, acc = a + 6;
                Q[p, p] = Qbar[p, p] * dt5;
                Q[p, v] = Q[v, p] = Qbar[p, v] * dt4;
                Q[p, acc] = Q[acc, p] = Qbar[p, acc] * dt3;
                Q[v, v] = Qbar[v, v] * dt3;
                Q[v, acc] = Qbar[acc, v] = Qbar[v, acc] * dt2;
                Q[acc, acc] = Qbar[acc, acc] * dt;
            }
            Q.Multiply(sigmaJ * sigmaJ, Q);
        }

        private void NormalizeInnovationAngles(Vector<double> inno)
        {
            inno[1] = NormalizeAngle(inno[1]);
            inno[2] = NormalizeAngle(inno[2]);
        }

        private void AdaptJerk(Vector<double> z)
        {
            double nis = ComputeNIS(z, R);
            if (nis > 9.0) sigmaJ = Math.Min(sigmaJ * 1.1, 10.0 * sigmaJ0);
            else if (nis < 3.0) sigmaJ = Math.Max(sigmaJ * 0.9, sigmaJ0);
        }

        private double ComputeNIS(Vector<double> z, Matrix<double> Rcust)
        {
            var inno = z - MeasurementFunction(State);
            NormalizeInnovationAngles(inno);

            ComputeJacobian(State, Hbuf);
            var S = Hbuf * Covariance * Hbuf.Transpose();
            S.Add(Rcust, S);

            var Sinv = S.Cholesky().Solve(DenseMatrix.CreateIdentity(MeasDim));
            return inno * (Sinv * inno);
        }

        private static double NormalizeAngle(double a) => Math.IEEERemainder(a, 2.0 * Math.PI);
        private static Matrix<double> Symmetrize(Matrix<double> M) => 0.5 * (M + M.Transpose());
    }
}

using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RealRadarSim.Tracking;

namespace RealRadarSim.Missile
{
    /// <summary>
    /// Very simple missile model using proportional navigation guidance.
    /// This is meant for ground launched Fox‑1 (semi active) missiles.
    /// The state is [position, velocity]. The missile has a limited burn time
    /// and a crude drag model. Gravity is accounted for.
    /// </summary>
    public class Fox1Missile
    {
        public Vector<double> Pos;  // [x,y,z] in meters
        public Vector<double> Vel;  // [vx,vy,vz] in m/s
        public bool Active => lifeTime < maxLifeTime && Pos[2] > 0;
        public bool HitTarget { get; private set; } = false;

        private readonly double mass0;        // initial mass (kg)
        private readonly double thrust;       // constant thrust (N)
        private readonly double burnTime;     // burn duration (s)
        private readonly double CdA;          // drag coefficient*area (m^2)
        private readonly double navConstant;  // PN constant (typically 3-5)
        private readonly double maxAccel;     // max achievable accel (m/s^2)
        private readonly double maxLifeTime;  // self destruct time

        private double lifeTime = 0.0;

        private const double rho = 1.225;     // air density (kg/m^3)
        private const double g = 9.81;

        private JPDA_Track targetTrack;

        public Fox1Missile(Vector<double> startPos, JPDA_Track track,
                           double navConstant = 4.0)
        {
            Pos = startPos.Clone();
            Vel = DenseVector.OfArray(new double[] { 0, 0, 0 });
            targetTrack = track;

            mass0 = 200.0;     // kg
            thrust = 20000.0;  // N
            burnTime = 5.0;    // sec
            CdA = 0.08;        // ~20 cm diameter
            this.navConstant = navConstant;
            maxAccel = 30.0 * g; // 30 g
            maxLifeTime = 60.0;  // seconds
        }

        private static Vector<double> Cross(Vector<double> a, Vector<double> b)
        {
            return DenseVector.OfArray(new double[]
            {
                a[1]*b[2]-a[2]*b[1],
                a[2]*b[0]-a[0]*b[2],
                a[0]*b[1]-a[1]*b[0]
            });
        }

        public void Update(double dt)
        {
            if (targetTrack == null) return;

            Vector<double> targetPos = targetTrack.Filter.State.SubVector(0, 3);
            Vector<double> targetVel = targetTrack.Filter.State.SubVector(3, 3);

            Vector<double> r = targetPos - Pos;
            Vector<double> vRel = targetVel - Vel;

            double rNorm2 = Math.Max(r.Dot(r), 1e-6);
            Vector<double> pnAccel = Cross(Vel, Cross(r, vRel)) * (navConstant / rNorm2);

            if (pnAccel.L2Norm() > maxAccel)
                pnAccel = pnAccel.Normalize(maxAccel);

            double speed = Vel.L2Norm();
            double mass = mass0; // no burn rate modelling
            Vector<double> unitVel = speed > 1e-6 ? Vel / speed : DenseVector.OfArray(new double[]{0,0,1});

            Vector<double> thrustAcc = (lifeTime < burnTime) ? (thrust / mass) * unitVel : DenseVector.Create(3, 0.0);
            Vector<double> dragAcc = -0.5 * rho * speed * speed * CdA / mass * unitVel;
            Vector<double> gravity = DenseVector.OfArray(new double[] { 0, 0, -g });

            Vector<double> totalAcc = pnAccel + thrustAcc + dragAcc + gravity;

            Vel += totalAcc * dt;
            Pos += Vel * dt;
            lifeTime += dt;

            if (r.L2Norm() < 20.0)
                HitTarget = true;
        }
    }
}

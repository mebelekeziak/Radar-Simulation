using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Distributions;

namespace RealRadarSim.Models
{
    public class TargetCT
    {
        // State: [x, y, z, speed, heading (rad), climbRate]
        public Vector<double> State;
        private double turnRateDeg;
        private double processStd;
        private Random rng;
        public string AircraftName { get; private set; }
        public double RCS { get; private set; }

        public TargetCT(double x, double y, double z,
            double speed, double headingDeg, double climbRate,
            double turnRateDeg, double processStd,
            string aircraftName, double rcs,
            Random rng)
        {
            State = DenseVector.OfArray(new double[]
            {
                x, y, z,
                speed,
                headingDeg * Math.PI / 180.0,
                climbRate
            });
            this.turnRateDeg = turnRateDeg;
            this.processStd = processStd;
            this.rng = rng;
            this.AircraftName = aircraftName;
            this.RCS = rcs;
        }

        public void Update(double dt)
        {
            double turnRateRad = turnRateDeg * Math.PI / 180.0;
            double randTurn = Normal.Sample(rng, turnRateRad, processStd * 0.2);
            double randClimb = Normal.Sample(rng, State[5], processStd);
            double newHeading = State[4] + randTurn * dt;
            double speed = State[3];
            double climb = randClimb;
            double vx = speed * Math.Cos(newHeading);
            double vy = speed * Math.Sin(newHeading);
            State[0] += vx * dt;
            State[1] += vy * dt;
            State[2] += climb * dt;
            State[3] = speed;
            State[4] = newHeading;
            State[5] = climb;
        }
    }
}

using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Distributions;

namespace RealRadarSim.Models
{
    /// <summary>
    /// Target with a Constant-Turn/Climb model.
    /// State = [x, y, z, speed, headingRad, climbRate].
    /// heading is stored in radians internally
    /// </summary>
    public class TargetCT
    {
        // [x, y, z, speed, headingRad, climbRate]
        public Vector<double> State;
        private double turnRateDeg;
        private double processStd;
        private Random rng;

        public string AircraftName { get; private set; }
        public double RCS { get; private set; }

        public TargetCT(
            double x, double y, double z,
            double speed, double headingDeg, double climbRate,
            double turnRateDeg, double processStd,
            string aircraftName, double rcs,
            Random rng)
        {
            State = DenseVector.OfArray(new double[]
            {
                x, y, z,
                speed,
                headingDeg * Math.PI / 180.0,  // store heading in radians
                climbRate
            });
            this.turnRateDeg = turnRateDeg;
            this.processStd = processStd;
            this.rng = rng;
            this.AircraftName = aircraftName;
            this.RCS = rcs;
        }

        /// <summary>
        /// Updates the target state by dt (seconds).
        /// </summary>
        public void Update(double dt)
        {
            // turnRate in deg -> rad
            double turnRateRad = turnRateDeg * Math.PI / 180.0;

            // Add a bit of process noise
            double randTurn = Normal.Sample(rng, turnRateRad, processStd * 0.2);
            double randClimb = Normal.Sample(rng, State[5], processStd);

            // heading
            double newHeading = State[4] + randTurn * dt;
            double speed = State[3];
            double climb = randClimb;

            // velocity in x/y
            double vx = speed * Math.Cos(newHeading);
            double vy = speed * Math.Sin(newHeading);

            // Integrate
            State[0] += vx * dt;    // x
            State[1] += vy * dt;    // y
            State[2] += climb * dt; // z
            State[3] = speed;
            State[4] = newHeading;
            State[5] = climb;
        }
    }
}

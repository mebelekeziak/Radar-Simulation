using System;

namespace RealRadarSim.Utils
{
    /// <summary>
    /// Common small‑angle helpers which previously appeared in multiple classes.
    /// </summary>
    public static class MathUtil
    {
        /// <summary>Degrees → radians.</summary>
        public static double DegToRad(double deg) => deg * Math.PI / 180.0;

        /// <summary>Radians → degrees.</summary>
        public static double RadToDeg(double rad) => rad * 180.0 / Math.PI;

        /// <summary>
        /// Wrap any radian angle into the open interval ( –π, π].
        /// </summary>
        public static double NormalizeAngle(double angleRad) => Math.IEEERemainder(angleRad, 2.0 * Math.PI);
    }
}
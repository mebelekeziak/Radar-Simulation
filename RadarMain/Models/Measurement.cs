namespace RealRadarSim.Models
{
    /// <summary>
    /// Radar measurement for one detection (range, azimuth, elevation, amplitude)
    /// </summary>
    public class Measurement
    {
        public double Range;
        public double Azimuth;
        public double Elevation;
        public double Amplitude;
        public double RadialVelocity;
        public string TargetName;
    }
}

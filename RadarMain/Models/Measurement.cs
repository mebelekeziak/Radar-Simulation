namespace RealRadarSim.Models
{
    /// very funny variables
    public class Measurement
    {
        public double Range;
        public double Azimuth;
        public double Elevation;
        public double Amplitude;
        public double RadialVelocity;
        public string TargetName; 
        // Signal-to-noise ratio (dB) at detection time; used for adaptive noise/gating.
        public double SNR_dB;
    }
}

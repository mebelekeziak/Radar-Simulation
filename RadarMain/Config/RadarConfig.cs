using System;

namespace RealRadarSim.Config
{
    /// <summary>
    /// Plain‑old C# object representing a complete radar configuration section
    /// from <c>config.lua</c>.  All fields are immutable once the record is built.
    /// </summary>
    public record RadarConfig
    {
        // Core geometry / scanning
        public double MaxRange { get; init; } = 100_000;
        public double BeamWidthDeg { get; init; } = 10.0;
        public double RotationSpeedDegSec { get; init; } = 36.0;
        public double FalseAlarmDensity { get; init; } = 1e-10;

        // SNR & RF parameters
        public double Snr0_dB { get; init; } = 30.0;
        public double ReferenceRange { get; init; } = 10_000.0;
        public double RequiredSNR_dB { get; init; } = 0.0;
        public double LockSNRThreshold_dB { get; init; } = 5.0;
        public double PathLossExponent_dB { get; init; } = 40.0;

        // Noise and CFAR
        public double RangeNoiseBase { get; init; } = 100.0;
        public double AngleNoiseBase { get; init; } = 0.001;
        public double CfarWindowWidth { get; init; } = 5_000.0;
        public double CfarGuardWidth { get; init; } = 300.0;
        public double CfarThresholdMultiplier { get; init; } = 8.0;
        public double ClusterDistanceMeters { get; init; } = 600.0;

        // Platform / antenna
        public string RadarType { get; init; } = "ground";
        public int AntennaElevationBars { get; init; } = 1;
        public double AntennaAzimuthScan { get; init; } = 140.0;
        public double TiltOffsetDeg { get; init; } = 0.0;

        // Lock & tracking aids
        public double LockRange { get; init; } = 50_000.0;

        // RF front‑end
        public double FrequencyHz { get; init; } = 3e9;
        public double TxPower_dBm { get; init; } = 70.0;
        public double AntennaGain_dBi { get; init; } = 101.0;

        // Visualisation toggles
        public bool ShowAzimuthBars { get; init; } = false;
        public bool ShowElevationBars { get; init; } = false;

        // Doppler
        public bool UseDopplerProcessing { get; init; } = false;
        public double VelocityNoiseStd { get; init; } = 1.0;
        public bool UseDopplerCFAR { get; init; } = false;
        public double DopplerCFARWindow { get; init; } = 150.0;
        public double DopplerCFARGuard { get; init; } = 20.0;
        public double DopplerCFARThresholdMultiplier { get; init; } = 6.0;

        // AESA / mode flags
        public bool UseAesaMode { get; init; } = false;

        /// <summary>Built‑in safe defaults used when no Lua file exists or it
        /// fails validation.
        /// </summary>
        public static RadarConfig Default => new();
    }
}
using System;
using System.Collections.Generic;
using RealRadarSim.Logging;
using RealRadarSim.Tracking;
using RealRadarSim.Utils;

namespace RealRadarSim.Models
{
    public partial class AdvancedRadar
    {
        public double MaxRange { get; private set; }
        public double BeamWidthRad { get; private set; }
        private readonly double rotationSpeedRadSec;
        private readonly double falseAlarmDensity;
        private readonly double snr0_dB;
        private readonly double requiredSNR_dB;
        private readonly double lockSNRThreshold_dB;
        private readonly double referenceRange;
        private const double referenceRCS = 1.0;
        private readonly double pathLossExponent_dB;
        private readonly double rangeNoiseBase;
        private readonly double angleNoiseBase;
        public double TiltOffsetDeg { get; set; } = 0.0;
        private readonly Random rng;
        public double CFARWindowWidth { get; private set; }
        public double CFARGuardWidth { get; private set; }
        public double CFARThresholdMultiplier { get; private set; }
        public double ClusterDistanceMeters { get; private set; }
        public double CurrentBeamAngle { get; private set; }
        public double CurrentAzimuth { get; private set; }
        public double CurrentElevation { get; private set; }
        public string RadarType { get; private set; } = "aircraft";
        public int AntennaHeight { get; set; } = 1;
        public bool ShowAzimuthBars { get; set; } = false;
        public bool ShowElevationBars { get; set; } = false;
        public double AntennaAzimuthScanDegrees { get; set; } = 140.0;
        public double SNR0_dB => snr0_dB;
        public double ReferenceRange => referenceRange;
        public double PathLossExponent_dB => pathLossExponent_dB;
        public double ReferenceRCS => referenceRCS;
        private double minAzimuth;
        private double maxAzimuth;
        private double minElevation;
        private double maxElevation;
        private int currentElevationBar;
        public int CurrentElevationBar => currentElevationBar;
        private bool scanLeftToRight;
        private double lockRange = 50_000.0;
        public bool UseAesaMode { get; set; } = false;
        public bool UseReferenceSNRModel { get; set; } = false;
        public int ConcurrentAesaBeams { get; set; } = 12;

        // ———————————————————————————————————  AESA compatibility stubs  ————————————————
        /// <summary>
        /// Multiplies rotational speed when AESA mode is enabled.  
        /// Exposed for backward compatibility; default = 1 (no speed change).
        /// </summary>
        public double BeamSpeedMultiplier { get; set; } = 1.0;

        /// <summary>
        /// Dummy per‑beam elevation oscillation frequency (rad/s).  
        /// Kept as a field because legacy code references it directly.
        /// </summary>
        private double aesaElevationOscFreq = 0.0;
        public double AesaElevationOscFreq
        {
            get => aesaElevationOscFreq;
            set => aesaElevationOscFreq = value;
        }

        // ———————————————————————————————————  backward compatibility  ——————————————————
        public List<AesaBeam> AesaBeams { get; private set; }

        public bool IsLocked => lockedTrack is not null;
        private JPDA_Track lockedTrack = null;
        public bool UseDopplerProcessing { get; set; } = false;
        public double VelocityNoiseStd { get; set; } = 1.0;
        public bool UseDopplerCFAR { get; set; } = false;
        public double DopplerCFARWindow { get; set; } = 150.0;
        public double DopplerCFARGuard { get; set; } = 20.0;
        public double DopplerCFARThresholdMultiplier { get; set; } = 6.0;
        public double DopplerCFARWindowWidth => DopplerCFARWindow;
        public double DopplerCFARGuardWidth => DopplerCFARGuard;
        public double FrequencyHz { get; set; } = 3e9;
        public double TxPower_dBm { get; set; } = 70.0;
        public double AntennaGain_dBi { get; set; } = 101.0;
        public double SystemLoss_dB { get; set; } = 3.0;
        public double AtmosphericLossOneWay_dB { get; set; } = 1.0;
        public double WeatherLossOneWay_dB { get; set; } = 0.5;

        // ——————————————————————————————  AESA Beam placeholder class  ——————————————————
        public class AesaBeam
        {
            public double CurrentAzimuth { get; set; }
            public double CurrentElevation { get; set; }

            public AesaBeam(double initialAz = 0, double initialEl = 0, double azP = 0, double elP = 0)
            {
                CurrentAzimuth = initialAz;
                CurrentElevation = initialEl;
            }

            public void Update(double dt, double rotSpeed, double elOsc) { }
        }

        // ------------------------------------------------‑ ctor ------------------------------------------------
        public AdvancedRadar(
            double maxRange,
            double beamWidthDeg,
            double rotationSpeedDegSec,
            double falseAlarmDensity,
            double snr0_dB,
            double referenceRange,
            double requiredSNR_dB,
            double rangeNoiseBase,
            double angleNoiseBase,
            Random rng,
            double cfarWindowWidth = 5_000.0,
            double cfarGuardWidth = 800.0,
            double cfarThresholdMultiplier = 8.0,
            double clusterDistanceMeters = 1_600.0,
            string radarType = "aircraft",
            int antennaHeight = 1,
            double antennaAzimuthScanDeg = 140.0,
            double tiltOffsetDeg = 0.0,
            double lockRange = 50_000.0,
            double lockSNRThreshold_dB = 5.0,
            double pathLossExponent_dB = 40.0,
            double frequencyHz = 3e9,
            double txPower_dBm = 70.0,
            double antennaGain_dBi = 101.0)
        {
            MaxRange = maxRange;
            BeamWidthRad = MathUtil.DegToRad(beamWidthDeg);
            rotationSpeedRadSec = MathUtil.DegToRad(rotationSpeedDegSec);
            this.falseAlarmDensity = falseAlarmDensity;
            this.snr0_dB = snr0_dB;
            this.referenceRange = referenceRange;
            this.requiredSNR_dB = requiredSNR_dB;
            this.lockSNRThreshold_dB = lockSNRThreshold_dB;
            this.pathLossExponent_dB = pathLossExponent_dB;
            this.rangeNoiseBase = rangeNoiseBase;
            this.angleNoiseBase = angleNoiseBase;
            this.rng = rng;
            CFARWindowWidth = cfarWindowWidth;
            CFARGuardWidth = cfarGuardWidth;
            CFARThresholdMultiplier = cfarThresholdMultiplier;
            ClusterDistanceMeters = clusterDistanceMeters;
            RadarType = radarType.ToLower();
            AntennaHeight = Math.Clamp(antennaHeight, 1, 6);
            AntennaAzimuthScanDegrees = antennaAzimuthScanDeg;
            TiltOffsetDeg = tiltOffsetDeg;
            this.lockRange = lockRange;
            FrequencyHz = frequencyHz;
            TxPower_dBm = txPower_dBm;
            AntennaGain_dBi = antennaGain_dBi;

            InitializeAircraftMode();

            // Preserve AESA API but disable backend functionality
            AesaBeams = new List<AesaBeam>();
        }

        // ------------- misc initialisation helpers ------------------
        private void InitializeAircraftMode()
        {
            double barSpacingRad = MathUtil.DegToRad(2.0);
            double tiltOffsetRad = MathUtil.DegToRad(TiltOffsetDeg);
            double halfSpanRad = (AntennaHeight - 1) * barSpacingRad * 0.5;
            minElevation = tiltOffsetRad - halfSpanRad;
            maxElevation = tiltOffsetRad + halfSpanRad;
            currentElevationBar = 0;
            CurrentElevation = minElevation;

            double halfAzRad = MathUtil.DegToRad(AntennaAzimuthScanDegrees * 0.5);
            minAzimuth = -halfAzRad;
            maxAzimuth = halfAzRad;
            CurrentAzimuth = minAzimuth;
            scanLeftToRight = true;
        }
    }
}

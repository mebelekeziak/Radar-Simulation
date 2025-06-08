using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using RealRadarSim.Logging;
using RealRadarSim.Tracking;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;

namespace RealRadarSim.Models
{
    public partial class AdvancedRadar
    {
        public double MaxRange { get; private set; }
        public double BeamWidthRad { get; private set; }

        private readonly double rotationSpeedRadSec;
        private readonly double falseAlarmDensity;

        // SNR parameters in dB
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

        // This value (in degrees) defines the total azimuth scan range.
        public double AntennaAzimuthScanDegrees { get; set; } = 140.0;

        public double SNR0_dB => snr0_dB;
        public double ReferenceRange => referenceRange;
        public double PathLossExponent_dB => pathLossExponent_dB;
        public double ReferenceRCS => 1.0;

        private double minAzimuth;
        private double maxAzimuth;
        private double minElevation;
        private double maxElevation;

        private int currentElevationBar;
        public int CurrentElevationBar => currentElevationBar;

        private bool scanLeftToRight;
        private double lockRange = 50000.0;

        // AESA mode flag.
        public bool UseAesaMode { get; set; } = false;

        public bool UseReferenceSNRModel { get; set; } = false;

        // New AESA mode properties.
        public int ConcurrentAesaBeams { get; set; } = 12;
        public List<AesaBeam> AesaBeams { get; private set; }
        private AesaScheduler scheduler;
        public AesaBeam LockBeam { get; private set; } = null;
        private double aesaElevationOscFreq = 0.1; // Hz

        /// <summary>
        /// Frequency of the AESA search beam elevation oscillation (Hz).
        /// </summary>
        public double AesaElevationOscFreq
        {
            get => aesaElevationOscFreq;
            set => aesaElevationOscFreq = Math.Max(0.01, value);
        }

        /// <summary>
        /// Multiplier controlling how quickly AESA beams slew compared to
        /// mechanical mode.
        /// </summary>
        public double BeamSpeedMultiplier { get; set; } = 5.0;

        // LOCK-RELATED FIELDS
        private JPDA_Track lockedTrack = null;
        public bool IsLocked => (lockedTrack != null);

        /// <summary>
        /// Enable or disable Doppler measurements entirely.
        /// </summary>
        public bool UseDopplerProcessing { get; set; } = false;

        /// <summary>
        /// Standard deviation of velocity measurement noise (m/s).
        /// </summary>
        public double VelocityNoiseStd { get; set; } = 1.0;

        // Doppler CFAR parameters
        public bool UseDopplerCFAR { get; set; } = false;
        public double DopplerCFARWindow { get; set; } = 150.0;   // m/s, for example
        public double DopplerCFARGuard { get; set; } = 20.0;     // m/s
        public double DopplerCFARThresholdMultiplier { get; set; } = 6.0;

        /// <summary>
        /// Alias for DopplerCFARWindow, used by DopplerCFARFilterMeasurements()
        // </summary>
        public double DopplerCFARWindowWidth => DopplerCFARWindow;

        /// <summary>
        /// Alias for DopplerCFARGuard, used by DopplerCFARFilterMeasurements()
        // </summary>
        public double DopplerCFARGuardWidth => DopplerCFARGuard;

        /// Path loss
        /// <summary>
        /// Operating frequency of the radar (Hz).
        /// </summary>
        public double FrequencyHz { get; set; } = 3e9; // 10 GHz default

        /// <summary>
        /// Transmit power in dBm (for demonstration).
        /// </summary>
        public double TxPower_dBm { get; set; } = 70.0; // e.g. 100 W ~ 50 dBm

        /// <summary>
        /// Antenna gain in dBi.
        /// </summary>
        public double AntennaGain_dBi { get; set; } = 101.0;

        /// <summary>
        /// System or miscellaneous losses in dB.
        /// </summary>
        public double SystemLoss_dB { get; set; } = 3.0;

        /// <summary>
        /// Atmospheric attenuation in dB (one-way).
        /// </summary>
        public double AtmosphericLossOneWay_dB { get; set; } = 1.0;

        /// <summary>
        /// Weather attenuation in dB (one-way).
        /// </summary>
        public double WeatherLossOneWay_dB { get; set; } = 0.5;

        /// <summary>
        /// Public helper that lets external test code run the built-in range-domain CA-CFAR
        /// on an arbitrary list of measurements without having to duplicate the algorithm.
        /// </summary>
        public List<Measurement> RunRangeCFAR(List<Measurement> measurements)
            => CFARFilterMeasurements(measurements);

        /// <summary>
        /// Public helper for the Doppler-domain CA-CFAR.
        /// </summary>
        public List<Measurement> RunDopplerCFAR(List<Measurement> measurements)
            => DopplerCFARFilterMeasurements(measurements);

        // Nested class representing a single AESA beam.
        public class AesaBeam
        {
            public double CurrentAzimuth; // in radians
            public double CurrentElevation; // in radians
            private double azPhase;
            private double elPhase;
            public JPDA_Track AssignedTrack { get; private set; } = null;

            public bool IsTracking => AssignedTrack != null;

            public AesaBeam(double initialAz, double initialEl, double initialAzPhase, double initialElPhase)
            {
                CurrentAzimuth = initialAz;
                CurrentElevation = initialEl;
                azPhase = initialAzPhase;
                elPhase = initialElPhase;
            }

            public void AssignTrack(JPDA_Track trk)
            {
                AssignedTrack = trk;
            }

            public void ClearAssignment()
            {
                AssignedTrack = null;
            }

            private static double Normalize(double a)
            {
                return Math.IEEERemainder(a, 2.0 * Math.PI);
            }

            public void Update(double dt, double rotationSpeed, double elevationOscFreq)
            {
                if (IsTracking)
                {
                    double x = AssignedTrack.Filter.State[0];
                    double y = AssignedTrack.Filter.State[1];
                    double z = AssignedTrack.Filter.State[2];

                    double desiredAz = Math.Atan2(y, x);
                    double desiredEl = Math.Atan2(z, Math.Sqrt(x * x + y * y));

                    double maxStep = rotationSpeed * dt;

                    double dAz = Normalize(desiredAz - CurrentAzimuth);
                    if (Math.Abs(dAz) > maxStep)
                        CurrentAzimuth += Math.Sign(dAz) * maxStep;
                    else
                        CurrentAzimuth = desiredAz;
                    CurrentAzimuth = Normalize(CurrentAzimuth);

                    double dEl = Normalize(desiredEl - CurrentElevation);
                    if (Math.Abs(dEl) > maxStep)
                        CurrentElevation += Math.Sign(dEl) * maxStep;
                    else
                        CurrentElevation = desiredEl;
                    CurrentElevation = Normalize(CurrentElevation);
                }
                else
                {
                    azPhase += rotationSpeed * dt;
                    elPhase += 2 * Math.PI * elevationOscFreq * dt;

                    // Oscillate azimuth between -70° and 70°
                    double minAz = -70.0 * Math.PI / 180.0;
                    double maxAz = 70.0 * Math.PI / 180.0;
                    double midAz = (minAz + maxAz) / 2.0;
                    double ampAz = (maxAz - minAz) / 2.0;
                    CurrentAzimuth = midAz + ampAz * Math.Sin(azPhase);

                    // Oscillate elevation between -15° and 15°
                    double minEl = -15.0 * Math.PI / 180.0;
                    double maxEl = 15.0 * Math.PI / 180.0;
                    double midEl = (minEl + maxEl) / 2.0;
                    double ampEl = (maxEl - minEl) / 2.0;
                    CurrentElevation = midEl + ampEl * Math.Sin(elPhase);
                }
            }
        }

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
            double cfarWindowWidth = 5000.0,
            double cfarGuardWidth = 800.0,
            double cfarThresholdMultiplier = 8.0,
            double clusterDistanceMeters = 1600.0,
            string radarType = "aircraft",
            int antennaHeight = 1,
            double antennaAzimuthScanDeg = 140.0,
            double tiltOffsetDeg = 0.0,
            double lockRange = 50000.0,
            double lockSNRThreshold_dB = 5.0,
            double pathLossExponent_dB = 2.0,
            double frequencyHz = 3e9,
            double txPower_dBm = 70.0,
            double antennaGain_dBi = 101.0
        )
        {
            MaxRange = maxRange;
            BeamWidthRad = beamWidthDeg * Math.PI / 180.0;
            rotationSpeedRadSec = rotationSpeedDegSec * Math.PI / 180.0;
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
            AntennaHeight = Math.Max(1, Math.Min(antennaHeight, 6));
            AntennaAzimuthScanDegrees = antennaAzimuthScanDeg;
            TiltOffsetDeg = tiltOffsetDeg;
            this.lockRange = lockRange;
            this.FrequencyHz = frequencyHz;
            this.TxPower_dBm = txPower_dBm;
            this.AntennaGain_dBi = antennaGain_dBi;

            InitializeAircraftMode();

            // Initialize AESA beams if in AESA mode.
            if (RadarType == "aircraft" && UseAesaMode)
            {
                AesaBeams = new List<AesaBeam>();
                for (int i = 0; i < ConcurrentAesaBeams; i++)
                {
                    double initialAzPhase = (2 * Math.PI / ConcurrentAesaBeams) * i;
                    double initialElPhase = (2 * Math.PI / ConcurrentAesaBeams) * i;
                    double minAz = -70.0 * Math.PI / 180.0;
                    double maxAz = 70.0 * Math.PI / 180.0;
                    double midAz = (minAz + maxAz) / 2.0;
                    double minEl = -15.0 * Math.PI / 180.0;
                    double maxEl = 15.0 * Math.PI / 180.0;
                    double midEl = (minEl + maxEl) / 2.0;
                    AesaBeams.Add(new AesaBeam(midAz, midEl, initialAzPhase, initialElPhase));
                }

                LockBeam = new AesaBeam(0, 0, 0, 0);
                scheduler = new AesaScheduler(this);
            }
        }

        private void InitializeAircraftMode()
        {
            double barSpacingDeg = 2.0;
            double barSpacingRad = barSpacingDeg * Math.PI / 180.0;
            double tiltOffsetRad = TiltOffsetDeg * Math.PI / 180.0;
            double totalElevDeg = (AntennaHeight - 1) * barSpacingDeg;
            double halfSpanRad = (totalElevDeg * 0.5) * (Math.PI / 180.0);

            minElevation = tiltOffsetRad - halfSpanRad;
            maxElevation = tiltOffsetRad + halfSpanRad;
            currentElevationBar = 0;
            CurrentElevation = minElevation;

            double halfAzDeg = AntennaAzimuthScanDegrees * 0.5;
            minAzimuth = -halfAzDeg * Math.PI / 180.0;
            maxAzimuth = halfAzDeg * Math.PI / 180.0;
            CurrentAzimuth = minAzimuth;
            scanLeftToRight = true;
        }

        public void LockTarget(JPDA_Track track)
        {
            if (RadarType == "aircraft")
            {
                lockedTrack = track;
                if (UseAesaMode && LockBeam != null)
                {
                    LockBeam.AssignTrack(track);
                }
            }
        }

        public void UnlockTarget()
        {
            lockedTrack = null;
            if (UseAesaMode && LockBeam != null)
            {
                LockBeam.ClearAssignment();
            }
        }

        public JPDA_Track GetLockedTrack() => lockedTrack;

        public void UpdateTrackAssignments(List<JPDA_Track> tracks)
        {
            if (!UseAesaMode || LockBeam == null)
                return;

            if (lockedTrack != null)
            {
                // already assigned by LockTarget
                return;
            }

            var best = tracks.OrderByDescending(t => t.ExistenceProb).FirstOrDefault();
            if (best != null && best.ExistenceProb > 0.6)
                LockBeam.AssignTrack(best);
            else
                LockBeam.ClearAssignment();

            scheduler?.AssignBeams(tracks);
        }

        public void UpdateBeam(double dt)
        {
            double effectiveMultiplier = UseAesaMode ? BeamSpeedMultiplier : 1.0;

            if (RadarType == "aircraft")
            {
                if (UseAesaMode)
                {
                    // Update search beams
                    if (AesaBeams != null)
                    {
                        foreach (var beam in AesaBeams)
                        {
                            beam.Update(dt, rotationSpeedRadSec * effectiveMultiplier, aesaElevationOscFreq);
                        }
                    }

                    // Update lock beam if assigned
                    if (LockBeam != null && LockBeam.IsTracking)
                    {
                        LockBeam.Update(dt, rotationSpeedRadSec * effectiveMultiplier, aesaElevationOscFreq);
                    }
                }
                else
                {
                    // Prioritize tracking mechanically when not in AESA mode
                    if (lockedTrack != null)
                    {
                        TrackLockedTrack(dt);
                        return;
                    }

                    double dAz = rotationSpeedRadSec * effectiveMultiplier * dt * (scanLeftToRight ? 1.0 : -1.0);
                    CurrentAzimuth += dAz;

                    if (scanLeftToRight && CurrentAzimuth >= maxAzimuth)
                    {
                        CurrentAzimuth = maxAzimuth;
                        scanLeftToRight = false;
                        AdvanceElevationBar();
                    }
                    else if (!scanLeftToRight && CurrentAzimuth <= minAzimuth)
                    {
                        CurrentAzimuth = minAzimuth;
                        scanLeftToRight = true;
                        AdvanceElevationBar();
                    }
                }
            }
            else
            {
                CurrentBeamAngle += rotationSpeedRadSec * effectiveMultiplier * dt;
                if (CurrentBeamAngle > 2 * Math.PI)
                    CurrentBeamAngle -= 2 * Math.PI;
            }
        }

        private void AdvanceElevationBar()
        {
            currentElevationBar++;
            if (currentElevationBar >= AntennaHeight)
                currentElevationBar = 0;

            double barSpacingDeg = 2.0;
            double barSpacingRad = barSpacingDeg * Math.PI / 180.0;
            CurrentElevation = minElevation + currentElevationBar * barSpacingRad;
        }

        private void TrackLockedTrack(double dt)
        {
            double x = lockedTrack.Filter.State[0];
            double y = lockedTrack.Filter.State[1];
            double z = lockedTrack.Filter.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);

            if (r > lockRange)
            {
                UnlockTarget();
                return;
            }

            double desiredAz = Math.Atan2(y, x);
            double desiredEl = Math.Atan2(z, Math.Sqrt(x * x + y * y));
            double maxAngularStep = rotationSpeedRadSec * dt;

            double deltaAz = NormalizeAngle(desiredAz - CurrentAzimuth);
            if (Math.Abs(deltaAz) > maxAngularStep)
                CurrentAzimuth += Math.Sign(deltaAz) * maxAngularStep;
            else
                CurrentAzimuth = desiredAz;
            CurrentAzimuth = NormalizeAngle(CurrentAzimuth);

            double deltaEl = NormalizeAngle(desiredEl - CurrentElevation);
            if (Math.Abs(deltaEl) > maxAngularStep)
                CurrentElevation += Math.Sign(deltaEl) * maxAngularStep;
            else
                CurrentElevation = desiredEl;
            CurrentElevation = NormalizeAngle(CurrentElevation);
        }

        public List<Measurement> GetMeasurements(List<TargetCT> targets)
        {
            var rawMeas = new List<Measurement>();
            int targetMeasurementCount = 0;
            foreach (var tgt in targets)
            {
                var m = GenerateTargetMeasurement(tgt);
                if (m != null)
                {
                    rawMeas.Add(m);
                    targetMeasurementCount++;
                }
            }
            DebugLogger.LogMeasurement($"Generated {targetMeasurementCount} target measurements.");
            double sectorArea = 0.5 * BeamWidthRad * MaxRange * MaxRange;
            double lambda = falseAlarmDensity * sectorArea;
            int numFalse = Poisson.Sample(rng, lambda);
            for (int i = 0; i < numFalse; i++)
            {
                rawMeas.Add(GenerateFalseAlarm());
            }
            var cfarDetections = CFARFilterMeasurements(rawMeas);
            DebugLogger.LogCFAR($"CFAR Filter: {cfarDetections.Count} detections passed out of {rawMeas.Count} raw measurements.");
            return MergeCloseDetections(cfarDetections, ClusterDistanceMeters);
        }

        public double GetSNR_dB(double rcs, double range)
        {
            return ComputeAdvancedRadarEquationSNR(rcs, range);
        }


        private Measurement GenerateTargetMeasurement(TargetCT tgt)
        {
            double x = tgt.State[0];
            double y = tgt.State[1];
            double z = tgt.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);
            if (r > MaxRange) return null;

            double az = Math.Atan2(y, x);
            double el = Math.Atan2(z, Math.Sqrt(x * x + y * y));

            // Beam shape expands at short ranges so that we don't "over-narrow" the beam
            double nominalBeamWidth = BeamWidthRad;
            double maxEffectiveBeamWidth = 30.0 * Math.PI / 180.0;
            double effectiveBeamWidth = nominalBeamWidth;
            if (r < referenceRange)
            {
                effectiveBeamWidth = nominalBeamWidth * (referenceRange / r);
                if (effectiveBeamWidth > maxEffectiveBeamWidth)
                    effectiveBeamWidth = maxEffectiveBeamWidth;
            }

            // Check if target is within the beam
            if (RadarType == "aircraft")
            {
                if (UseAesaMode)
                {
                    bool inAnyBeam = false;

                    if (LockBeam != null && LockBeam.IsTracking)
                    {
                        double diffAzL = Math.Abs(NormalizeAngle(az - LockBeam.CurrentAzimuth));
                        double diffElL = Math.Abs(NormalizeAngle(el - LockBeam.CurrentElevation));
                        double effElWidth = effectiveBeamWidth * 2.0;
                        if (diffAzL <= effectiveBeamWidth * 0.5 && diffElL <= effElWidth * 0.5)
                            inAnyBeam = true;
                    }

                    if (!inAnyBeam && AesaBeams != null)
                    {
                        foreach (var beam in AesaBeams)
                        {
                            double diffAz = Math.Abs(NormalizeAngle(az - beam.CurrentAzimuth));
                            if (diffAz <= effectiveBeamWidth * 0.5)
                            {
                                double effectiveElevationWidth = effectiveBeamWidth * 2.0;
                                double diffEl = Math.Abs(NormalizeAngle(el - beam.CurrentElevation));
                                if (diffEl <= effectiveElevationWidth * 0.5)
                                {
                                    inAnyBeam = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!inAnyBeam) return null;
                }
                else
                {
                    double diffAz = Math.Abs(NormalizeAngle(az - CurrentAzimuth));
                    if (diffAz > effectiveBeamWidth * 0.5) return null;

                    // Single bar approach in elevation
                    const double singleBarDeg = 2.0;
                    double barHalfRad = (singleBarDeg * Math.PI / 180.0) * 0.5;
                    double diffEl = Math.Abs(NormalizeAngle(el - CurrentElevation));
                    if (diffEl > barHalfRad) return null;
                }
            }
            else
            {
                // Typical rotating ground-based radar
                double diffBeam = Math.Abs(NormalizeAngle(az - CurrentBeamAngle));
                if (diffBeam > effectiveBeamWidth * 0.5) return null;
            }

            // Compute SNR from a more advanced radar equation (still somewhat simplified).
            double snr_dB = ComputeAdvancedRadarEquationSNR(tgt.RCS, r);
            if (UseAesaMode)
            {
                // Might get an AESA advantage in power or scanning flexibility
                snr_dB += 3.0;
            }

            if (snr_dB < requiredSNR_dB)
                return null;

            // Add measurement noise
            double rMeas = r + Normal.Sample(rng, 0, rangeNoiseBase);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = az + Normal.Sample(rng, 0, angleNoiseBase);
            double elMeas = el + Normal.Sample(rng, 0, angleNoiseBase);

            // Convert SNR back to linear scale and add random perturbation
            double snr_linear = Math.Pow(10.0, snr_dB / 10.0);
            // Adjust amplitude if beam is effectively widened/narrowed
            snr_linear *= (nominalBeamWidth / effectiveBeamWidth);
            double amp = snr_linear + Normal.Sample(rng, 0, 0.05 * snr_linear);

            // Optionally compute Doppler (radial velocity)
            double radialVel = 0.0;
            if (UseDopplerProcessing && tgt.State.Count >= 6)
            {
                double vx = tgt.State[3];
                double vy = tgt.State[4];
                double vz = tgt.State[5];
                radialVel = (x * vx + y * vy + z * vz) / (r + 1e-6); // dot(los, velocity)
                radialVel += Normal.Sample(rng, 0, VelocityNoiseStd);
            }

            var measurement = new Measurement
            {
                Range = rMeas,
                Azimuth = NormalizeAngle(azMeas),
                Elevation = NormalizeAngle(elMeas),
                Amplitude = amp,
                RadialVelocity = radialVel
            };

            DebugLogger.LogMeasurement(
                $"Generated target measurement: R = {measurement.Range:F2}, Az = {measurement.Azimuth:F2}, " +
                $"El = {measurement.Elevation:F2}, Amp = {measurement.Amplitude:F2}, Vel = {measurement.RadialVelocity:F2}"
            );
            return measurement;
        }

        /// <summary>
        /// Returns post-integration SNR (dB) for a target with the given RCS at the
        /// specified range.  Uses either a quick reference model or the full radar
        /// equation with thermal noise.
        /// </summary>
        private double ComputeAdvancedRadarEquationSNR(double rcs, double range)
        {
            if (UseReferenceSNRModel)
            {
                double otherLosses_dB =
                    2.0 * (AtmosphericLossOneWay_dB + WeatherLossOneWay_dB) +
                    SystemLoss_dB;

                double rangeRatio_dB = 10.0 * Math.Log10(range / referenceRange);
                double rcsRatio_dB = 10.0 * Math.Log10(rcs / ReferenceRCS);

                return snr0_dB
                     - (pathLossExponent_dB * rangeRatio_dB)
                     + rcsRatio_dB
                     - otherLosses_dB;
            }

            // --- FULL MONOSTATIC RADAR EQUATION -------------------------------------
            const double c = 3.0e8;        // speed of light (m/s)
            const double k_dBW = -228.6;   // Boltzmann constant 10·log₁₀(k)  (dBW/K/Hz)
            const double T0_dB = 24.6;     // 10·log₁₀(290 K)
            const double bandwidth_dB = 60.0;      // 1 MHz IF bandwidth
            const double noiseFigure_dB = 5.0;     // receiver NF
            const double pulseCount = 10.0;        // coherent pulses integrated

            // 1) Signal power at the receiver
            double lambda = c / FrequencyHz;                 // wavelength (m)
            double txPower_dBW = TxPower_dBm - 30.0;         // dBm → dBW
            double totalGain_dB = 2.0 * AntennaGain_dBi;     // G² term (in dB)
            double rcs_dB = 10.0 * Math.Log10(rcs);          // σ (dB)

            // Constant and range-dependent path loss terms
            double const_dB = 20.0 * Math.Log10(lambda)         // +20 log λ
                               - 30.0 * Math.Log10(4.0 * Math.PI); // −30 log 4π
            double range_dB = -40.0 * Math.Log10(range);        // −40 log R

            double totalLosses_dB =
                2.0 * (AtmosphericLossOneWay_dB + WeatherLossOneWay_dB) +
                SystemLoss_dB;

            double signalPower_dBW =
                txPower_dBW
              + totalGain_dB
              + const_dB
              + rcs_dB
              + range_dB
              - totalLosses_dB;

            // 2) Thermal noise power
            double noisePower_dBW = k_dBW + T0_dB + bandwidth_dB + noiseFigure_dB;

            // 3) Non-coherent integration gain (10·log₁₀ N)
            double integrationGain_dB = 10.0 * Math.Log10(pulseCount);

            // 4) SNR = S – N + G_int
            return signalPower_dBW - noisePower_dBW + integrationGain_dB;
        }


        private Measurement GenerateFalseAlarm()
        {
            double u = rng.NextDouble();
            double rFA = MaxRange * Math.Sqrt(u);

            double halfBeam = BeamWidthRad * 0.5;
            double mainAz;
            if (RadarType == "aircraft" && UseAesaMode)
            {
                if (LockBeam != null && LockBeam.IsTracking && rng.NextDouble() < 0.3)
                {
                    mainAz = LockBeam.CurrentAzimuth;
                }
                else if (AesaBeams != null && AesaBeams.Count > 0)
                {
                    int idx = rng.Next(AesaBeams.Count);
                    mainAz = AesaBeams[idx].CurrentAzimuth;
                }
                else
                {
                    mainAz = CurrentAzimuth;
                }
            }
            else
            {
                mainAz = (RadarType == "aircraft") ? CurrentAzimuth : CurrentBeamAngle;
            }

            double azFA = NormalizeAngle(mainAz + (rng.NextDouble() * BeamWidthRad - halfBeam));
            double elCenter;
            if (RadarType == "aircraft" && UseAesaMode)
            {
                if (LockBeam != null && LockBeam.IsTracking && rng.NextDouble() < 0.3)
                    elCenter = LockBeam.CurrentElevation;
                else if (AesaBeams != null && AesaBeams.Count > 0)
                    elCenter = AesaBeams[rng.Next(AesaBeams.Count)].CurrentElevation;
                else
                    elCenter = CurrentElevation;
            }
            else
            {
                elCenter = (RadarType == "aircraft") ? CurrentElevation : 0.0;
            }
            double elFA = elCenter + Normal.Sample(rng, 0, angleNoiseBase * 2);

            double rMeas = rFA + Normal.Sample(rng, 0, rangeNoiseBase * 0.5);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = NormalizeAngle(azFA + Normal.Sample(rng, 0, angleNoiseBase));
            double elMeas = NormalizeAngle(elFA + Normal.Sample(rng, 0, angleNoiseBase));

            // Basic false alarm amplitude
            double falseAmp_linear = Math.Pow(10.0, requiredSNR_dB / 10.0)
                + Normal.Sample(rng, 0, 0.2 * Math.Pow(10.0, requiredSNR_dB / 10.0));
            if (falseAmp_linear < 0.0)
                falseAmp_linear = 0.8 * Math.Pow(10.0, requiredSNR_dB / 10.0);

            // Doppler for false alarms can be random if Doppler is enabled
            double radialVel = 0.0;
            if (UseDopplerProcessing)
            {
                radialVel = Normal.Sample(rng, 0, 100.0); // random false alarm velocity
            }

            return new Measurement
            {
                Range = rMeas,
                Azimuth = azMeas,
                Elevation = elMeas,
                Amplitude = falseAmp_linear,
                RadialVelocity = radialVel
            };
        }

        /// <summary>
        /// Range‑domain CA‑CFAR with a sliding window O(n) implementation.
        /// Assumes <paramref name="measurements"/> is unsorted.
        /// </summary>
        private List<Measurement> CFARFilterMeasurements(List<Measurement> measurements)
        {
            if (measurements.Count == 0) return measurements;
        
            // 1) Sort by range once.
            var sorted = measurements
                .OrderBy(m => m.Range)
                .ToArray();
        
            int n = sorted.Length;
            bool[] pass = new bool[n];
            var results = new List<Measurement>(n);
        
            // 2) Sliding window indices.
            int left = 0, right = 0;
        
            // Pre‑compute linearised amplitudes to avoid repeat Pow().
            double[] amp = new double[n];
            for (int i = 0; i < n; ++i) amp[i] = sorted[i].Amplitude;
        
            for (int i = 0; i < n; ++i)
            {
                double rCut = sorted[i].Range;
        
                // Expand right edge of the window.
                while (right < n && sorted[right].Range - rCut <= CFARWindowWidth)
                    right++;
        
                // Shrink left edge (guard band).
                while (left < i && rCut - sorted[left].Range > CFARWindowWidth)
                    left++;
        
                // Collect training cells excluding guard band.
                var noise = new List<double>();
                for (int k = left; k < right; ++k)
                {
                    if (k == i) continue;                          // skip CUT
                    if (Math.Abs(sorted[k].Range - rCut) <= CFARGuardWidth) continue; // skip guard
                    noise.Add(amp[k]);
                }
        
                double threshold;
                if (noise.Count == 0)
                {
                    // Fallback: 1.5 × required SNR (linear)
                    threshold = 1.5 * Math.Pow(10.0, requiredSNR_dB / 10.0);
                }
                else
                {
                    // 75‑percentile of training set – robust to outliers
                    noise.Sort();
                    double noiseEstimate = noise[(int)(0.75 * (noise.Count - 1))];
                    threshold = CFARThresholdMultiplier * noiseEstimate;
                }
        
                pass[i] = amp[i] >= threshold;
                if (pass[i]) results.Add(sorted[i]);
            }
        
            return results;
        }
        
        
        private List<Measurement> DopplerCFARFilterMeasurements(List<Measurement> measurements)
        {
            if (measurements.Count == 0)
                return measurements;
        
            // 1) Sort by radial velocity once.
            var sorted = measurements
                .OrderBy(m => m.RadialVelocity)
                .ToArray();
        
            int n = sorted.Length;
            var results = new List<Measurement>(n);
        
            // Sliding‐window bounds.
            int left = 0, right = 0;
        
            // Pre‑compute amplitudes to avoid repeated Pow() calls.
            double[] amp = new double[n];
            for (int i = 0; i < n; ++i)
                amp[i] = sorted[i].Amplitude;
        
            for (int i = 0; i < n; ++i)
            {
                // Current CUT velocity
                double vCut = sorted[i].RadialVelocity;
        
                // Expand right edge while within the velocity window
                while (right < n && sorted[right].RadialVelocity - vCut <= DopplerCFARWindowWidth)
                    right++;
        
                // Shrink left edge past the window
                while (left < i && vCut - sorted[left].RadialVelocity > DopplerCFARWindowWidth)
                    left++;
        
                // Gather training cells (exclude guard band and CUT)
                var noise = new List<double>(capacity: right - left);
                for (int k = left; k < right; ++k)
                {
                    if (k == i) 
                        continue;                                  // skip CUT
                    if (Math.Abs(sorted[k].RadialVelocity - vCut) <= DopplerCFARGuardWidth) 
                        continue;                                  // skip guard cells
                    noise.Add(amp[k]);
                }
        
                // Compute threshold
                double threshold;
                if (noise.Count == 0)
                {
                    // No training cells → fallback to a fixed SNR in linear scale
                    threshold = 1.5 * Math.Pow(10.0, requiredSNR_dB / 10.0);
                }
                else
                {
                    // 75th‐percentile noise estimate → robust thresholding
                    noise.Sort();
                    int idx = (int)(0.75 * (noise.Count - 1));
                    double noiseEstimate = noise[idx];
                    threshold = DopplerCFARThresholdMultiplier * noiseEstimate;
                }
        
                // If CUT exceeds threshold, keep it
                if (amp[i] >= threshold)
                    results.Add(sorted[i]);
            }
        
            return results;
        }

        /// <summary>
        /// Agglomerates any detections belonging to the same physical target.
        /// A fast union–find clusters in Cartesian space, but the distance
        /// threshold grows with range so that even at 100 km two echoes inside
        /// the beam collapse to one.
        /// </summary>
        private List<Measurement> MergeCloseDetections(List<Measurement> meas, double baseDist)
        {
            int n = meas.Count;
            if (n <= 1) return meas;

            // ----- build union‑find -----
            int[] parent = new int[n];
            for (int i = 0; i < n; ++i) parent[i] = i;

            int Find(int x) => parent[x] == x ? x : parent[x] = Find(parent[x]);
            void Union(int a, int b)
            {
                a = Find(a); b = Find(b);
                if (a != b) parent[b] = a;
            }

            // pair‑wise loop (n is usually tiny, so O(n²) is fine)
            for (int i = 0; i < n - 1; ++i)
            {
                for (int j = i + 1; j < n; ++j)
                {
                    // dynamic threshold: take the larger range of the pair
                    double rMax = Math.Max(meas[i].Range, meas[j].Range);

                    // angular footprint of the main‑lobe at that range
                    double angTol = 0.8 * BeamWidthRad * rMax;         // 80 % of beam footprint
                    double rangeTol = baseDist;                         // given in ctor (e.g. 600 m)

                    // choose the tighter of the two metrics
                    double dCart = CartesianDistance(meas[i], meas[j]);
                    if (dCart < Math.Max(rangeTol, angTol))
                        Union(i, j);
                }
            }

            // ----- pick one representative per cluster (highest amplitude) -----
            Dictionary<int, Measurement> repr = new Dictionary<int, Measurement>();
            for (int i = 0; i < n; ++i)
            {
                int root = Find(i);
                if (!repr.ContainsKey(root) || meas[i].Amplitude > repr[root].Amplitude)
                    repr[root] = meas[i];
            }

            return repr.Values.ToList();
        }

        private double CartesianDistance(Measurement a, Measurement b)
        {
            var (ax, ay, az) = ToCartesian(a);
            var (bx, by, bz) = ToCartesian(b);

            double dx = ax - bx;
            double dy = ay - by;
            double dz = az - bz;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private (double x, double y, double z) ToCartesian(Measurement m)
        {
            double x = m.Range * Math.Cos(m.Elevation) * Math.Cos(m.Azimuth);
            double y = m.Range * Math.Cos(m.Elevation) * Math.Sin(m.Azimuth);
            double z = m.Range * Math.Sin(m.Elevation);
            return (x, y, z);
        }

        private double NormalizeAngle(double angle)
        {
            // Wrap any radian angle into (–π, π] in O(1) time
            return Math.IEEERemainder(angle, 2.0 * Math.PI);
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using RealRadarSim.Logging;

namespace RealRadarSim.Models
{
    public class AdvancedRadar
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
        private bool aesaScanLeftToRight = true;

        // AESA mode parameters.
        public bool UseAesaMode { get; set; } = false;
        private double aesaTime = 0.0;
        private double aesaElevationOscFreq = 0.1; // Hz

        // NEW: Multiply the rotation speed to make the beam faster.
        public double BeamSpeedMultiplier { get; set; } = 5.0;

        // LOCK-RELATED FIELDS
        private TargetCT lockedTarget = null;
        public bool IsLocked => (lockedTarget != null);
        private double lockRange = 50000.0;

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
            double cfarGuardWidth = 300.0,
            double cfarThresholdMultiplier = 8.0,
            double clusterDistanceMeters = 600.0,
            string radarType = "aircraft",
            int antennaHeight = 1,
            double antennaAzimuthScanDeg = 140.0,
            double tiltOffsetDeg = 0.0,
            double lockRange = 50000.0,
            double lockSNRThreshold_dB = 5.0,
            double pathLossExponent_dB = 40.0
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

            InitializeAircraftMode();
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

        public void LockTarget(TargetCT target)
        {
            if (RadarType == "aircraft")
            {
                lockedTarget = target;
            }
        }

        public void UnlockTarget()
        {
            lockedTarget = null;
        }

        public void UpdateBeam(double dt)
        {
            if (RadarType == "aircraft")
            {
                if (UseAesaMode)
                {
                    aesaTime += dt;
                    double halfAesaRad = (AntennaAzimuthScanDegrees / 2.0) * Math.PI / 180.0;
                    double aesaMinAz = -halfAesaRad;
                    double aesaMaxAz = halfAesaRad;
                    if (aesaScanLeftToRight)
                    {
                        // NEW: Multiply the speed by BeamSpeedMultiplier.
                        CurrentAzimuth += rotationSpeedRadSec * BeamSpeedMultiplier * dt;
                        if (CurrentAzimuth >= aesaMaxAz)
                        {
                            CurrentAzimuth = aesaMaxAz;
                            aesaScanLeftToRight = false;
                        }
                    }
                    else
                    {
                        CurrentAzimuth -= rotationSpeedRadSec * BeamSpeedMultiplier * dt;
                        if (CurrentAzimuth <= aesaMinAz)
                        {
                            CurrentAzimuth = aesaMinAz;
                            aesaScanLeftToRight = true;
                        }
                    }
                    double aesaMinEl = -15.0 * Math.PI / 180.0;
                    double aesaMaxEl = 15.0 * Math.PI / 180.0;
                    CurrentElevation = aesaMinEl + (aesaMaxEl - aesaMinEl) * 0.5 * (1 + Math.Sin(2 * Math.PI * aesaElevationOscFreq * aesaTime));
                }
                else
                {
                    if (lockedTarget != null)
                    {
                        TrackLockedTarget(dt);
                    }
                    else
                    {
                        double dAz = rotationSpeedRadSec * BeamSpeedMultiplier * dt * (scanLeftToRight ? 1.0 : -1.0);
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
            }
            else
            {
                CurrentBeamAngle += rotationSpeedRadSec * BeamSpeedMultiplier * dt;
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

        private void TrackLockedTarget(double dt)
        {
            double x = lockedTarget.State[0];
            double y = lockedTarget.State[1];
            double z = lockedTarget.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);

            double snr_dB = snr0_dB
                + 10.0 * Math.Log10(lockedTarget.RCS / referenceRCS)
                + 40.0 * Math.Log10(referenceRange / r);

            try
            {
                string logEntry = $"Range: {r:F0} m, SNR(dB): {snr_dB:F1}, LockThreshold(dB): {lockSNRThreshold_dB}{Environment.NewLine}";
                System.IO.File.AppendAllText("debug_log.txt", logEntry);
            }
            catch (Exception)
            {
            }

            if (r > lockRange || snr_dB < lockSNRThreshold_dB)
            {
                UnlockTarget();
                return;
            }

            double desiredAz = Math.Atan2(y, x);
            double desiredEl = Math.Atan2(z, Math.Sqrt(x * x + y * y));
            double maxAngularStep = rotationSpeedRadSec * dt;

            double deltaAz = NormalizeAngle(desiredAz - CurrentAzimuth);
            if (Math.Abs(deltaAz) > maxAngularStep)
            {
                CurrentAzimuth += Math.Sign(deltaAz) * maxAngularStep;
            }
            else
            {
                CurrentAzimuth = desiredAz;
            }
            CurrentAzimuth = NormalizeAngle(CurrentAzimuth);

            double deltaEl = NormalizeAngle(desiredEl - CurrentElevation);
            if (Math.Abs(deltaEl) > maxAngularStep)
            {
                CurrentElevation += Math.Sign(deltaEl) * maxAngularStep;
            }
            else
            {
                CurrentElevation = desiredEl;
            }
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

        private Measurement GenerateTargetMeasurement(TargetCT tgt)
        {
            double x = tgt.State[0];
            double y = tgt.State[1];
            double z = tgt.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);
            if (r > MaxRange) return null;

            double az = Math.Atan2(y, x);
            double el = Math.Atan2(z, Math.Sqrt(x * x + y * y));

            double nominalBeamWidth = BeamWidthRad;
            double maxEffectiveBeamWidth = 30.0 * Math.PI / 180.0;
            double effectiveBeamWidth = nominalBeamWidth;
            if (r < referenceRange)
            {
                effectiveBeamWidth = nominalBeamWidth * (referenceRange / r);
                if (effectiveBeamWidth > maxEffectiveBeamWidth)
                    effectiveBeamWidth = maxEffectiveBeamWidth;
            }

            if (RadarType == "aircraft")
            {
                if (UseAesaMode)
                {
                    double diffAz = Math.Abs(NormalizeAngle(az - CurrentAzimuth));
                    if (diffAz > effectiveBeamWidth * 0.5) return null;
                    double effectiveElevationWidth = effectiveBeamWidth * 2.0;
                    double diffEl = Math.Abs(NormalizeAngle(el - CurrentElevation));
                    if (diffEl > effectiveElevationWidth * 0.5) return null;
                }
                else
                {
                    double diffAz = Math.Abs(NormalizeAngle(az - CurrentAzimuth));
                    if (diffAz > effectiveBeamWidth * 0.5) return null;
                    const double singleBarDeg = 2.0;
                    double barHalfRad = (singleBarDeg * Math.PI / 180.0) * 0.5;
                    double diffEl = Math.Abs(NormalizeAngle(el - CurrentElevation));
                    if (diffEl > barHalfRad) return null;
                }
            }
            else
            {
                double diffBeam = Math.Abs(NormalizeAngle(az - CurrentBeamAngle));
                if (diffBeam > effectiveBeamWidth * 0.5) return null;
            }

            double snr_dB = snr0_dB
                + 10.0 * Math.Log10(tgt.RCS / referenceRCS)
                + pathLossExponent_dB * Math.Log10(referenceRange / r);
            if (UseAesaMode)
            {
                snr_dB += 3.0;
            }
            if (snr_dB < requiredSNR_dB) return null;
            double snr_linear = Math.Pow(10.0, snr_dB / 10.0);
            snr_linear *= (nominalBeamWidth / effectiveBeamWidth);
            double rMeas = r + Normal.Sample(rng, 0, rangeNoiseBase);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = az + Normal.Sample(rng, 0, angleNoiseBase);
            double elMeas = el + Normal.Sample(rng, 0, angleNoiseBase);
            double amp = snr_linear + Normal.Sample(rng, 0, 0.05 * snr_linear);

            var measurement = new Measurement
            {
                Range = rMeas,
                Azimuth = NormalizeAngle(azMeas),
                Elevation = NormalizeAngle(elMeas),
                Amplitude = amp
            };

            DebugLogger.LogMeasurement($"Generated target measurement: Range = {measurement.Range:F2}, Azimuth = {measurement.Azimuth:F2}, Elevation = {measurement.Elevation:F2}, Amplitude = {measurement.Amplitude:F2}");
            return measurement;
        }

        private Measurement GenerateFalseAlarm()
        {
            double u = rng.NextDouble();
            double rFA = MaxRange * Math.Sqrt(u);
            double halfBeam = BeamWidthRad * 0.5;
            double mainAz = (RadarType == "aircraft") ? CurrentAzimuth : CurrentBeamAngle;
            double azFA = NormalizeAngle(mainAz + (rng.NextDouble() * BeamWidthRad - halfBeam));
            double elCenter = (RadarType == "aircraft") ? CurrentElevation : 0.0;
            double elFA = elCenter + Normal.Sample(rng, 0, angleNoiseBase * 2);
            double rMeas = rFA + Normal.Sample(rng, 0, rangeNoiseBase * 0.5);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = NormalizeAngle(azFA + Normal.Sample(rng, 0, angleNoiseBase));
            double elMeas = NormalizeAngle(elFA + Normal.Sample(rng, 0, angleNoiseBase));

            double falseAmp_linear = Math.Pow(10.0, requiredSNR_dB / 10.0)
                + Normal.Sample(rng, 0, 0.2 * Math.Pow(10.0, requiredSNR_dB / 10.0));
            if (falseAmp_linear < 0.0)
                falseAmp_linear = 0.8 * Math.Pow(10.0, requiredSNR_dB / 10.0);

            return new Measurement
            {
                Range = rMeas,
                Azimuth = azMeas,
                Elevation = elMeas,
                Amplitude = falseAmp_linear
            };
        }

        private List<Measurement> CFARFilterMeasurements(List<Measurement> measurements)
        {
            var sorted = measurements.OrderBy(m => m.Range).ToList();
            var results = new List<Measurement>();
            bool[] passed = new bool[sorted.Count];

            for (int i = 0; i < sorted.Count; i++)
            {
                Measurement cut = sorted[i];
                double cutRange = cut.Range;
                var training = new List<double>();

                foreach (var neighbor in sorted)
                {
                    if (ReferenceEquals(neighbor, cut)) continue;
                    double dr = Math.Abs(neighbor.Range - cutRange);
                    if (dr <= CFARWindowWidth && dr > CFARGuardWidth)
                        training.Add(neighbor.Amplitude);
                }

                DebugLogger.LogCFAR($"CFAR - Measurement index {i}: Range = {cutRange:F2}, Amplitude = {cut.Amplitude:F2}, Training data count = {training.Count}");

                if (training.Count == 0)
                {
                    double fallbackThreshold = Math.Pow(10.0, requiredSNR_dB / 10.0) * 1.5;
                    passed[i] = (cut.Amplitude > fallbackThreshold);
                    DebugLogger.LogCFAR($"CFAR - Measurement index {i}: No training data available. Fallback threshold (1.5 * required SNR linear) = {fallbackThreshold:F2}. Decision: {(passed[i] ? "Passed" : "Rejected")}.");
                    continue;
                }

                training.Sort();
                int K = (int)(0.75 * training.Count);
                if (K >= training.Count)
                    K = training.Count - 1;
                if (K < 0)
                    K = 0;
                double noiseEstimate = training[K];

                double threshold = CFARThresholdMultiplier * noiseEstimate;
                passed[i] = (cut.Amplitude >= threshold);

                DebugLogger.LogCFAR($"CFAR - Measurement index {i}: Sorted training list = [{string.Join(", ", training.Select(a => a.ToString("F2")))}], K index = {K}, Noise estimate = {noiseEstimate:F2}, CFARThresholdMultiplier = {CFARThresholdMultiplier}, Computed threshold = {threshold:F2}, Decision: {(passed[i] ? "Passed" : "Rejected")}.");
            }

            for (int i = 0; i < sorted.Count; i++)
            {
                if (passed[i])
                    results.Add(sorted[i]);
            }

            return results;
        }

        private List<Measurement> MergeCloseDetections(List<Measurement> meas, double maxDist)
        {
            var merged = new List<Measurement>();
            bool[] used = new bool[meas.Count];
            var sorted = meas.OrderByDescending(m => m.Amplitude).ToList();

            for (int i = 0; i < sorted.Count; i++)
            {
                if (used[i]) continue;
                var current = sorted[i];
                merged.Add(current);
                used[i] = true;

                for (int j = i + 1; j < sorted.Count; j++)
                {
                    if (used[j]) continue;
                    double dist = CartesianDistance(current, sorted[j]);
                    if (dist < maxDist)
                        used[j] = true;
                }
            }
            return merged;
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
            while (angle > Math.PI)
                angle -= 2.0 * Math.PI;
            while (angle < -Math.PI)
                angle += 2.0 * Math.PI;
            return angle;
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using RealRadarSim.Logging;

namespace RealRadarSim.Models
{
    public class AdvancedRadar
    {
        public enum RadarOperationMode
        {
            Mechanical,
            AESA
        }

        // ------------------------------
        // Nested class for AESA beams
        // ------------------------------
        private class Beam
        {
            public double Azimuth { get; set; }
            public double Elevation { get; set; }
            public double DwellTime { get; set; }
            public double TimeSinceLastUpdate { get; set; }
            public int DetectionCount { get; set; }
            public bool IsTrackBeam { get; set; }  // If this beam is used to track a locked target
        }

        // ------------------------------
        // Private Fields
        // ------------------------------
        private List<Beam> searchBeams = new List<Beam>(); // The standard search grid beams
        private Beam trackBeam = null;                     // If locked, we spawn a specialized track beam
        private int azBeamCount = 10;                      // Default: number of azimuth beams
        private int elBeamCount = 5;                       // Default: number of elevation beams
        private double baseDwellTime = 0.1;                // Base dwell time in seconds

        // For scanning in mechanical mode:
        private bool scanLeftToRight;
        private int currentElevationBar;  // The active bar index

        // For AESA scheduling:
        private double aesaTime = 0.0;    // Keeps running sum of time for AESA scheduling
        private int currentAesaBeamIndex = 0;

        // Radar range / geometry parameters
        public double MaxRange { get; private set; }
        public double BeamWidthRad { get; private set; }
        private readonly double rotationSpeedRadSec;
        private readonly double falseAlarmDensity;

        // SNR parameters (dB)
        private readonly double snr0_dB;
        private readonly double requiredSNR_dB;
        private readonly double lockSNRThreshold_dB;
        private readonly double referenceRange;
        private readonly double pathLossExponent_dB;

        private readonly double rangeNoiseBase;
        private readonly double angleNoiseBase;

        // CFAR and clustering
        public double CFARWindowWidth { get; private set; }
        public double CFARGuardWidth { get; private set; }
        public double CFARThresholdMultiplier { get; private set; }
        public double ClusterDistanceMeters { get; private set; }

        private double minAzimuth;
        private double maxAzimuth;
        private double minElevation;
        private double maxElevation;

        // Private random
        private readonly Random rng;

        public double TiltOffsetDeg { get; set; } = 0.0;
        public string RadarType { get; private set; } = "ground";
        public int AntennaHeight { get; set; } = 1;
        public bool ShowAzimuthBars { get; set; } = false;
        public bool ShowElevationBars { get; set; } = false;
        public double AntennaAzimuthScanDegrees { get; set; } = 140.0;

        // Public “exposed” read-only properties
        public double SNR0_dB => snr0_dB;
        public double ReferenceRange => referenceRange;
        public double PathLossExponent_dB => pathLossExponent_dB;
        public double ReferenceRCS => 1.0; // always 1 m^2
        public double MinAzimuth => minAzimuth;
        public double MaxAzimuth => maxAzimuth;
        public double AesaTime => aesaTime; // total time for AESA

        public int AzBeamCount => azBeamCount;
        public int ElBeamCount => elBeamCount;
        public double BaseDwellTime => baseDwellTime;

        // For mechanical scanning
        public double CurrentBeamAngle { get; private set; }
        public double CurrentAzimuth { get; private set; }
        public double CurrentElevation { get; private set; }

        // Elevation bar index for UI
        public int CurrentElevationBar => currentElevationBar;

        // LOCK-RELATED FIELDS
        private TargetCT lockedTarget = null;
        public bool IsLocked => (lockedTarget != null);
        private double lockRange = 50000.0;

        // Replaced auto-property with backing field implementation.
        private RadarOperationMode operationMode = RadarOperationMode.Mechanical;

        public RadarOperationMode OperationMode
        {
            get { return operationMode; }
            set
            {
                if (operationMode != value)
                {
                    operationMode = value;

                    // When switching to AESA mode and if the radar type is "aircraft",
                    // initialize the AESA beams.
                    if (RadarType.ToLower() == "aircraft" && operationMode == RadarOperationMode.AESA)
                    {
                        InitializeAesaSearchBeams();
                    }
                    // Optionally, if switching away from AESA, you might clear the beams:
                    else if (RadarType.ToLower() == "aircraft" && operationMode != RadarOperationMode.AESA)
                    {
                        searchBeams.Clear();
                    }
                }
            }
        }

        // ------------------------------
        // Public API
        // ------------------------------

        public void LockTarget(TargetCT target)
        {
            if (RadarType == "aircraft")
            {
                lockedTarget = target;
                // Optionally create/enable a track beam if in AESA mode:
                if (OperationMode == RadarOperationMode.AESA)
                {
                    CreateOrUpdateTrackBeam(lockedTarget);
                }
            }
        }

        public void UnlockTarget()
        {
            lockedTarget = null;
            trackBeam = null;
            currentAesaBeamIndex = 0; // Reset the index to ensure it’s within bounds.
        }

        // ------------------------------
        // Constructor
        // ------------------------------
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
            double cfarThresholdMultiplier = 0.00000000000005,
            double clusterDistanceMeters = 600.0,
            string radarType = "ground",
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

            if (RadarType == "aircraft")
            {
                InitializeAircraftMode();
                if (OperationMode == RadarOperationMode.AESA)
                {
                    InitializeAesaSearchBeams();
                }
            }
            else
            {
                // Ground-based: mechanical 360-scan
                CurrentBeamAngle = 0.0;
            }
        }

        private void InitializeAircraftMode()
        {
            // For mechanical bars
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

        private void InitializeAesaSearchBeams()
        {
            searchBeams.Clear();

            double azSectorWidth = maxAzimuth - minAzimuth;
            double azStep = (azBeamCount > 1) ? azSectorWidth / (azBeamCount - 1) : 0.0;

            // Example: we define elevation from 0 up to some max (say 10,000m vs. referenceRange).
            // You can also parametrize this if you want more bars.
            double minElevAESA = 0.0;
            double maxElevAESA = Math.Atan2(10000.0, referenceRange);
            double elevStep = (elBeamCount > 1) ? (maxElevAESA - minElevAESA) / (elBeamCount - 1) : 0.0;

            for (int i = 0; i < elBeamCount; i++)
            {
                for (int j = 0; j < azBeamCount; j++)
                {
                    Beam beam = new Beam
                    {
                        Azimuth = minAzimuth + j * azStep,
                        Elevation = minElevAESA + i * elevStep,
                        DwellTime = baseDwellTime,
                        TimeSinceLastUpdate = 0.0,
                        DetectionCount = 0,
                        IsTrackBeam = false
                    };
                    searchBeams.Add(beam);
                }
            }
        }

        private void CreateOrUpdateTrackBeam(TargetCT t)
        {
            // If locked, we define a single "track" beam that points at target’s direction.
            if (t == null) return;

            if (trackBeam == null)
            {
                trackBeam = new Beam
                {
                    Azimuth = 0.0,
                    Elevation = 0.0,
                    DwellTime = baseDwellTime, // can tune for tracking
                    DetectionCount = 0,
                    IsTrackBeam = true
                };
            }
        }

        // ------------------------------
        // Main update: scanning or tracking
        // ------------------------------
        public void UpdateBeam(double dt)
        {
            if (RadarType == "aircraft")
            {
                if (lockedTarget != null)
                {
                    // If locked, do STT-like behavior or track logic
                    TrackLockedTarget(dt);
                }
                else
                {
                    // If not locked, proceed with normal scanning
                    if (OperationMode == RadarOperationMode.AESA)
                    {
                        UpdateAesaBeams(dt);
                    }
                    else // mechanical
                    {
                        double dAz = rotationSpeedRadSec * dt * (scanLeftToRight ? 1.0 : -1.0);
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
                // Ground radar rotates continuously
                CurrentBeamAngle += rotationSpeedRadSec * dt;
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

        // ------------------------------
        // AESA Scheduling
        // ------------------------------
        private void UpdateAesaBeams(double dt)
        {
            // 1) Increment total AESA time.
            aesaTime += dt;

            // 2) Update track beam if locked.
            if (lockedTarget != null && trackBeam != null)
            {
                double x = lockedTarget.State[0];
                double y = lockedTarget.State[1];
                double z = lockedTarget.State[2];
                trackBeam.Azimuth = Math.Atan2(y, x);
                trackBeam.Elevation = Math.Atan2(z, Math.Sqrt(x * x + y * y));
            }

            // 3) Build the combined list of beams.
            var allBeams = new List<Beam>();
            allBeams.AddRange(searchBeams);
            if (trackBeam != null)
                allBeams.Add(trackBeam);

            // Safeguard: if there are no beams, exit.
            if (allBeams.Count == 0)
                return;

            // Ensure currentAesaBeamIndex is within bounds.
            currentAesaBeamIndex %= allBeams.Count;

            // 4) Compute the new active beam index.
            double totalCycleTime = baseDwellTime * allBeams.Count;
            double localTime = aesaTime % totalCycleTime;
            int newIndex = (int)(localTime / baseDwellTime);

            if (newIndex != currentAesaBeamIndex)
            {
                // Reset the previous beam’s counters.
                allBeams[currentAesaBeamIndex].DetectionCount = 0;
                allBeams[currentAesaBeamIndex].TimeSinceLastUpdate = 0.0;

                currentAesaBeamIndex = newIndex;
            }

            // 5) Set the active beam’s angles.
            Beam active = allBeams[currentAesaBeamIndex];
            CurrentAzimuth = active.Azimuth;
            CurrentElevation = active.Elevation;
        }


        public int GetCurrentAesaBeamIndex()
        {
            return currentAesaBeamIndex;
        }

        /// <summary>
        /// Returns the list of beams (for UI display): the first "searchBeams" in order, plus
        /// the trackBeam if any. The index of the active beam is <see cref="GetCurrentAesaBeamIndex"/>.
        /// </summary>
        public List<(double azRad, bool isTrack)> GetAesaBeams()
        {
            // Return them in the same order used by UpdateAesaBeams
            var list = searchBeams
                .Select(b => (b.Azimuth, b.IsTrackBeam))
                .ToList();
            if (trackBeam != null)
            {
                list.Add((trackBeam.Azimuth, trackBeam.IsTrackBeam));
            }
            return list;
        }

        // ------------------------------
        // Track-locked target logic
        // ------------------------------
        private void TrackLockedTarget(double dt)
        {
            double x = lockedTarget.State[0];
            double y = lockedTarget.State[1];
            double z = lockedTarget.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);

            double snr_dB = snr0_dB
                + 10.0 * Math.Log10(lockedTarget.RCS / ReferenceRCS)
                + 40.0 * Math.Log10(referenceRange / r);

            if (r > lockRange || snr_dB < lockSNRThreshold_dB)
            {
                // Lost lock
                UnlockTarget();
                return;
            }

            // For mechanical scanning track, we just smoothly steer the antenna:
            if (OperationMode == RadarOperationMode.Mechanical)
            {
                double desiredAz = Math.Atan2(y, x);
                double desiredEl = Math.Atan2(z, Math.Sqrt(x * x + y * y));

                double maxAngularStep = rotationSpeedRadSec * dt;

                double deltaAz = NormalizeAngle(desiredAz - CurrentAzimuth);
                if (Math.Abs(deltaAz) > maxAngularStep)
                    CurrentAzimuth += Math.Sign(deltaAz) * maxAngularStep;
                else
                    CurrentAzimuth = desiredAz;

                double deltaEl = NormalizeAngle(desiredEl - CurrentElevation);
                if (Math.Abs(deltaEl) > maxAngularStep)
                    CurrentElevation += Math.Sign(deltaEl) * maxAngularStep;
                else
                    CurrentElevation = desiredEl;
            }
            else
            {
                // In AESA mode, we rely on the trackBeam approach in UpdateAesaBeams.
                // So do nothing special here.
            }
        }

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }

        // ------------------------------
        // Measurement Generation
        // ------------------------------
        public List<Measurement> GetMeasurements(List<TargetCT> targets)
        {
            var rawMeas = new List<Measurement>();

            int targetMeasurementCount = 0;
            // Generate target returns
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

            // Generate false alarms
            double sectorArea = 0.5 * BeamWidthRad * MaxRange * MaxRange;
            double lambda = falseAlarmDensity * sectorArea;
            int numFalse = Poisson.Sample(rng, lambda);
            for (int i = 0; i < numFalse; i++)
            {
                rawMeas.Add(GenerateFalseAlarm());
            }

            // Apply CFAR filtering and clustering
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

            // Effective beam width for closer targets, etc.
            double nominalBeamWidth = BeamWidthRad;
            double maxEffectiveBeamWidth = 30.0 * Math.PI / 180.0;
            double effectiveBeamWidth = nominalBeamWidth;
            if (r < referenceRange)
            {
                effectiveBeamWidth = nominalBeamWidth * (referenceRange / r);
                if (effectiveBeamWidth > maxEffectiveBeamWidth)
                    effectiveBeamWidth = maxEffectiveBeamWidth;
            }

            bool targetDetected = false;
            if (OperationMode == RadarOperationMode.AESA)
            {
                // Check if within the "active" AESA beam for the current moment or any beam
                // Because we do time multiplexing, only the "active beam" matters at any instant.
                // But to keep it simpler, we check if target is in any of the beams in the grid.
                // This is a simplification for demonstration.
                var allBeams = new List<Beam>(searchBeams);
                if (trackBeam != null) allBeams.Add(trackBeam);

                foreach (var beam in allBeams)
                {
                    double diffAz = Math.Abs(NormalizeAngle(az - beam.Azimuth));
                    double barHalfRad = (2.0 * Math.PI / 180.0) * 0.5;
                    double diffEl = Math.Abs(NormalizeAngle(el - beam.Elevation));
                    if (diffAz <= effectiveBeamWidth * 0.5 && diffEl <= barHalfRad)
                    {
                        targetDetected = true;
                        beam.DetectionCount++;
                        break;
                    }
                }
                if (!targetDetected) return null;
            }
            else
            {
                // Mechanical: compare to CurrentAzimuth/CurrentElevation
                double diffAz = Math.Abs(NormalizeAngle(az - CurrentAzimuth));
                if (diffAz > effectiveBeamWidth * 0.5) return null;

                const double singleBarDeg = 2.0;
                double barHalfRad = (singleBarDeg * Math.PI / 180.0) * 0.5;
                double diffEl = Math.Abs(NormalizeAngle(el - CurrentElevation));
                if (diffEl > barHalfRad) return null;
            }

            // Check SNR
            double snr_dB = snr0_dB
                + 10.0 * Math.Log10(tgt.RCS / ReferenceRCS)
                + pathLossExponent_dB * Math.Log10(referenceRange / r);

            if (snr_dB < requiredSNR_dB) return null;

            // Convert to linear, scale by ratio of nominal/effective beam
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
            DebugLogger.LogMeasurement($"Generated target measurement: Range={measurement.Range:F2}, Az={measurement.Azimuth:F2}, El={measurement.Elevation:F2}, Amp={measurement.Amplitude:F2}");
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

                DebugLogger.LogCFAR($"CFAR - idx={i}: Range={cutRange:F2}, Amp={cut.Amplitude:F2}, #train={training.Count}");

                if (training.Count == 0)
                {
                    double fallbackThreshold = Math.Pow(10.0, requiredSNR_dB / 10.0) * 1.5;
                    passed[i] = (cut.Amplitude > fallbackThreshold);
                    DebugLogger.LogCFAR($"No training data -> fallback threshold={fallbackThreshold:F2}, pass={passed[i]}");
                    continue;
                }

                training.Sort();
                int K = (int)(0.75 * training.Count);
                if (K >= training.Count) K = training.Count - 1;
                if (K < 0) K = 0;

                double noiseEstimate = training[K];
                double threshold = CFARThresholdMultiplier * noiseEstimate;
                passed[i] = (cut.Amplitude >= threshold);
                DebugLogger.LogCFAR($"noiseEst={noiseEstimate:F2}, threshold={threshold:F2}, pass={passed[i]}");
            }

            for (int i = 0; i < sorted.Count; i++)
            {
                if (passed[i]) results.Add(sorted[i]);
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
    }
}

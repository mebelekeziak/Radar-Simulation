﻿using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;

namespace RealRadarSim.Models
{
    public class AdvancedRadar
    {
        public double MaxRange { get; private set; }
        public double BeamWidthRad { get; private set; }

        private readonly double rotationSpeedRadSec;
        private readonly double falseAlarmDensity;
        private readonly double snr0;
        private readonly double referenceRange;
        private readonly double requiredSNR;
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

        public string RadarType { get; private set; } = "ground";

        public int AntennaHeight { get; set; } = 1;
        public bool ShowAzimuthBars { get; set; } = false;
        public bool ShowElevationBars { get; set; } = false;

        public double AntennaAzimuthScanDegrees { get; set; } = 140.0;

        private double minAzimuth;
        private double maxAzimuth;
        private double minElevation;
        private double maxElevation;

        private int currentElevationBar;  // The active bar index
        private bool scanLeftToRight;

        // ADDED: Public read‐only property to expose the current elevation bar index.
        public int CurrentElevationBar => currentElevationBar;  // ADDED

        public AdvancedRadar(
            double maxRange,
            double beamWidthDeg,
            double rotationSpeedDegSec,
            double falseAlarmDensity,
            double snr0,
            double referenceRange,
            double requiredSNR,
            double rangeNoiseBase,
            double angleNoiseBase,
            Random rng,
            double cfarWindowWidth = 5000.0,
            double cfarGuardWidth = 300.0,
            double cfarThresholdMultiplier = 8.0,
            double clusterDistanceMeters = 600.0,
            string radarType = "ground",
            int antennaHeight = 1,
            double antennaAzimuthScanDeg = 140.0,
            double tiltOffsetDeg = 0.0
        )
        {
            MaxRange = maxRange;
            BeamWidthRad = beamWidthDeg * Math.PI / 180.0;
            rotationSpeedRadSec = rotationSpeedDegSec * Math.PI / 180.0;
            this.falseAlarmDensity = falseAlarmDensity;
            this.snr0 = snr0;
            this.referenceRange = referenceRange;
            this.requiredSNR = requiredSNR;
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

            if (RadarType == "aircraft")
            {
                InitializeAircraftMode();
            }
            else
            {
                CurrentBeamAngle = 0.0;
            }
        }

        private void InitializeAircraftMode()
        {
            // Example: each bar spaced by 2°
            double barSpacingDeg = 2.0;

            // Convert degrees to radians
            double barSpacingRad = barSpacingDeg * Math.PI / 180.0;

            // We want the total vertical coverage to be (AntennaHeight - 1) * barSpacingDeg
            // and centered around TiltOffsetDeg.
            double tiltOffsetRad = TiltOffsetDeg * Math.PI / 180.0;

            double totalElevDeg = (AntennaHeight - 1) * barSpacingDeg;
            double halfSpanRad = (totalElevDeg * 0.5) * (Math.PI / 180.0);

            // So minElevation is (tiltOffsetRad - halfSpanRad) 
            // and maxElevation is (tiltOffsetRad + halfSpanRad).
            minElevation = tiltOffsetRad - halfSpanRad;
            maxElevation = tiltOffsetRad + halfSpanRad;

            // Start scanning from the lowest bar.
            currentElevationBar = 0;
            CurrentElevation = minElevation;

            // Azimuth scanning from - half Az to + half Az
            double halfAzDeg = AntennaAzimuthScanDegrees * 0.5;
            minAzimuth = -halfAzDeg * Math.PI / 180.0;
            maxAzimuth = halfAzDeg * Math.PI / 180.0;

            CurrentAzimuth = minAzimuth;
            scanLeftToRight = true;
        }

        public void UpdateBeam(double dt)
        {
            if (RadarType == "aircraft")
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
            else
            {
                // 360° ground radar style
                CurrentBeamAngle += rotationSpeedRadSec * dt;
                if (CurrentBeamAngle > 2 * Math.PI)
                    CurrentBeamAngle -= 2 * Math.PI;
            }
        }

        private void AdvanceElevationBar()
        {
            // Move to the next bar index.
            currentElevationBar++;
            if (currentElevationBar >= AntennaHeight)
                currentElevationBar = 0;

            // For barSpacingDeg used above:
            double barSpacingDeg = 2.0;
            double barSpacingRad = barSpacingDeg * Math.PI / 180.0;

            // Recompute the bar’s elevation from the minElevation
            CurrentElevation = minElevation + currentElevationBar * barSpacingRad;
        }

        public List<Measurement> GetMeasurements(List<TargetCT> targets)
        {
            var rawMeas = new List<Measurement>();

            // Generate target returns
            foreach (var tgt in targets)
            {
                var m = GenerateTargetMeasurement(tgt);
                if (m != null) rawMeas.Add(m);
            }

            // Generate false alarms
            double sectorArea = 0.5 * BeamWidthRad * MaxRange * MaxRange;
            double lambda = falseAlarmDensity * sectorArea;
            int numFalse = Poisson.Sample(rng, lambda);
            for (int i = 0; i < numFalse; i++)
            {
                rawMeas.Add(GenerateFalseAlarm());
            }

            // CFAR & Clustering
            var cfarDetections = CFARFilterMeasurements(rawMeas);
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

            if (RadarType == "aircraft")
            {
                double diffAz = Math.Abs(NormalizeAngle(az - CurrentAzimuth));
                // Check if within azimuth beam
                if (diffAz > BeamWidthRad * 0.5) return null;

                // The code uses bar halfwidth of ~1° or 2° depending on your config
                const double singleBarDeg = 2.0;
                double barHalfRad = (singleBarDeg * Math.PI / 180.0) * 0.5;
                double diffEl = Math.Abs(NormalizeAngle(el - CurrentElevation));
                if (diffEl > barHalfRad) return null;
            }
            else
            {
                // ground radar
                double diffBeam = Math.Abs(NormalizeAngle(az - CurrentBeamAngle));
                if (diffBeam > BeamWidthRad * 0.5) return null;
            }

            // SNR check
            double snr = snr0 * (tgt.RCS / 10.0) * Math.Pow(referenceRange / r, 2);
            if (snr < requiredSNR) return null;

            // Measurement noise
            double rMeas = r + Normal.Sample(rng, 0, rangeNoiseBase);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = az + Normal.Sample(rng, 0, angleNoiseBase);
            double elMeas = el + Normal.Sample(rng, 0, angleNoiseBase);
            double amp = snr + Normal.Sample(rng, 0, 0.05 * snr);

            return new Measurement
            {
                Range = rMeas,
                Azimuth = NormalizeAngle(azMeas),
                Elevation = NormalizeAngle(elMeas),
                Amplitude = amp
            };
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
            double falseAmp = Normal.Sample(rng, requiredSNR, 0.2 * requiredSNR);
            if (falseAmp < 0.0) falseAmp = requiredSNR * 0.8;

            return new Measurement
            {
                Range = rMeas,
                Azimuth = azMeas,
                Elevation = elMeas,
                Amplitude = falseAmp
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

                if (training.Count == 0)
                {
                    passed[i] = (cut.Amplitude > (requiredSNR * 1.5));
                    continue;
                }
                training.Sort();
                int K = (int)(0.75 * training.Count);
                if (K >= training.Count) K = training.Count - 1;
                if (K < 0) K = 0;
                double noiseEstimate = training[K];
                double threshold = CFARThresholdMultiplier * noiseEstimate;
                passed[i] = (cut.Amplitude >= threshold);
            }

            for (int i = 0; i < sorted.Count; i++)
            {
                if (passed[i]) results.Add(sorted[i]);
            }

            double avgAmp = (results.Count > 0) ? results.Average(m => m.Amplitude) : 0.0;
            double minCutoff = Math.Max(requiredSNR * 0.3, 0.2 * avgAmp);

            return results.Where(m => m.Amplitude >= minCutoff).ToList();
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
                    if (dist < maxDist) used[j] = true;
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
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }
    }
}

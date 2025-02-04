using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using RealRadarSim.Models;

namespace RealRadarSim.Models
{
    public class AdvancedRadar
    {
        public double MaxRange { get; private set; }
        public double CurrentBeamAngle { get; private set; }

        // Radar parameters
        private readonly double beamWidthRad;
        private readonly double rotationSpeedRadSec;
        private readonly double falseAlarmDensity;
        private readonly double snr0;
        private readonly double referenceRange;
        private readonly double requiredSNR;
        private readonly double rangeNoiseBase;
        private readonly double angleNoiseBase;
        private readonly Random rng;

        // CFAR and clustering parameters
        public double CFARWindowWidth { get; private set; }
        public double CFARGuardWidth { get; private set; }
        public double CFARThresholdMultiplier { get; private set; }
        public double ClusterDistanceMeters { get; private set; }

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
            double clusterDistanceMeters = 600.0)
        {
            MaxRange = maxRange;
            beamWidthRad = beamWidthDeg * Math.PI / 180.0;
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
        }

        // CFAR filtering for the measurements.
        private List<Measurement> CFARFilterMeasurements(List<Measurement> measurements)
        {
            var sorted = measurements.OrderBy(m => m.Range).ToList();
            var results = new List<Measurement>(sorted.Count);
            bool[] passedCFAR = new bool[sorted.Count];

            for (int cutIdx = 0; cutIdx < sorted.Count; cutIdx++)
            {
                Measurement cut = sorted[cutIdx];
                double cutRange = cut.Range;
                var trainingCells = new List<double>();

                foreach (var neighbor in sorted)
                {
                    if (ReferenceEquals(neighbor, cut))
                        continue;
                    double dr = Math.Abs(neighbor.Range - cutRange);
                    if (dr <= CFARWindowWidth && dr > CFARGuardWidth)
                        trainingCells.Add(neighbor.Amplitude);
                }

                if (trainingCells.Count == 0)
                {
                    passedCFAR[cutIdx] = (cut.Amplitude > (requiredSNR * 1.5));
                    continue;
                }

                trainingCells.Sort();
                int K = (int)(0.75 * trainingCells.Count);
                if (K >= trainingCells.Count) K = trainingCells.Count - 1;
                if (K < 0) K = 0;

                double noiseEstimate = trainingCells[K];
                double threshold = CFARThresholdMultiplier * noiseEstimate;
                passedCFAR[cutIdx] = (cut.Amplitude >= threshold);
            }

            for (int i = 0; i < sorted.Count; i++)
            {
                if (passedCFAR[i])
                    results.Add(sorted[i]);
            }

            double avgAmp = results.Count > 0 ? results.Average(m => m.Amplitude) : 0.0;
            double minCutoff = Math.Max(requiredSNR * 0.3, 0.2 * avgAmp);
            var finalFiltered = results.Where(m => m.Amplitude >= minCutoff).ToList();
            return finalFiltered;
        }

        // Merge close detections to avoid duplicates.
        private List<Measurement> MergeCloseDetections(List<Measurement> measurements, double maxDistance)
        {
            var merged = new List<Measurement>();
            bool[] used = new bool[measurements.Count];
            var sorted = measurements.OrderByDescending(m => m.Amplitude).ToList();

            for (int i = 0; i < sorted.Count; i++)
            {
                if (used[i])
                    continue;
                Measurement current = sorted[i];
                merged.Add(current);
                used[i] = true;
                for (int j = i + 1; j < sorted.Count; j++)
                {
                    if (used[j])
                        continue;
                    double dist = CartesianDistance(current, sorted[j]);
                    if (dist < maxDistance)
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

        public void UpdateBeam(double dt)
        {
            CurrentBeamAngle += rotationSpeedRadSec * dt;
            if (CurrentBeamAngle > 2.0 * Math.PI)
                CurrentBeamAngle -= 2.0 * Math.PI;
        }

        public List<Measurement> GetMeasurements(List<TargetCT> targets)
        {
            var rawMeasurements = new List<Measurement>();

            // Generate target returns.
            foreach (var tgt in targets)
            {
                var meas = GenerateTargetMeasurement(tgt);
                if (meas != null)
                    rawMeasurements.Add(meas);
            }

            // Generate false alarms.
            double sectorArea = 0.5 * beamWidthRad * MaxRange * MaxRange;
            double lambda = falseAlarmDensity * sectorArea;
            int numFalse = Poisson.Sample(rng, lambda);
            for (int i = 0; i < numFalse; i++)
                rawMeasurements.Add(GenerateFalseAlarm());

            var cfarDetections = CFARFilterMeasurements(rawMeasurements);
            var merged = MergeCloseDetections(cfarDetections, ClusterDistanceMeters);
            return merged;
        }

        private Measurement GenerateTargetMeasurement(TargetCT tgt)
        {
            double x = tgt.State[0];
            double y = tgt.State[1];
            double z = tgt.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);
            if (r > MaxRange) return null;
            double az = Math.Atan2(y, x);
            double beamDiff = Math.Abs(NormalizeAngle(az - CurrentBeamAngle));
            if (beamDiff > beamWidthRad / 2.0) return null;
            double snr = snr0 * (tgt.RCS / 10.0) * Math.Pow(referenceRange / r, 2);
            if (snr < requiredSNR) return null;
            double rMeas = r + Normal.Sample(rng, 0, rangeNoiseBase);
            if (rMeas < 1.0) rMeas = 1.0;
            double angleError = Normal.Sample(rng, 0, angleNoiseBase);
            double azMeas = az + angleError;
            double rho = Math.Sqrt(x * x + y * y);
            if (rho < 1e-6) rho = 1e-6;
            double el = Math.Atan2(z, rho);
            double elMeas = el + Normal.Sample(rng, 0, angleNoiseBase);
            double measuredAmp = snr + Normal.Sample(rng, 0, 0.05 * snr);
            return new Measurement
            {
                Range = rMeas,
                Azimuth = NormalizeAngle(azMeas),
                Elevation = NormalizeAngle(elMeas),
                Amplitude = measuredAmp
            };
        }

        private Measurement GenerateFalseAlarm()
        {
            double u = rng.NextDouble();
            double rFA = MaxRange * Math.Sqrt(u);
            double halfBeam = beamWidthRad / 2.0;
            double azFA = NormalizeAngle(CurrentBeamAngle + (rng.NextDouble() * beamWidthRad - halfBeam));
            double elFA = Normal.Sample(rng, 0, angleNoiseBase);
            double rMeas = rFA + Normal.Sample(rng, 0, rangeNoiseBase * 0.5);
            if (rMeas < 1.0) rMeas = 1.0;
            double azMeas = NormalizeAngle(azFA + Normal.Sample(rng, 0, angleNoiseBase));
            double elMeas = NormalizeAngle(elFA + Normal.Sample(rng, 0, angleNoiseBase));
            double falseAmp = Normal.Sample(rng, requiredSNR * 1.0, 0.2 * requiredSNR);
            if (falseAmp < 0.0) falseAmp = requiredSNR * 0.8;
            return new Measurement
            {
                Range = rMeas,
                Azimuth = azMeas,
                Elevation = elMeas,
                Amplitude = falseAmp
            };
        }

        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }
    }
}

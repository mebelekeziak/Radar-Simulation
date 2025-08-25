using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.Distributions;
using RealRadarSim.Logging;
using RealRadarSim.Utils;

namespace RealRadarSim.Models
{
    public partial class AdvancedRadar
    {
        // ———————————————————————————————————  public surface  ——————————————————————————————————
        public List<Measurement> RunRangeCFAR(List<Measurement> m) => CFARFilterMeasurements(m);
        public List<Measurement> RunDopplerCFAR(List<Measurement> m) => DopplerCFARFilterMeasurements(m);

        public List<Measurement> GetMeasurements(List<TargetCT> targets)
        {
            var raw = new List<Measurement>();
            foreach (var t in targets)
            {
                if (GenerateTargetMeasurement(t) is { } m)
                    raw.Add(m);
            }
            double sectorArea = 0.5 * BeamWidthRad * MaxRange * MaxRange;
            int numFalse = Poisson.Sample(rng, falseAlarmDensity * sectorArea);
            for (int i = 0; i < numFalse; i++)
                raw.Add(GenerateFalseAlarm());
            var cfar = CFARFilterMeasurements(raw);
            return MergeCloseDetections(cfar, ClusterDistanceMeters);
        }

        // —————————————————————————  internal generation helpers  —————————————————————————
        private Measurement GenerateTargetMeasurement(TargetCT tgt)
        {
            double x = tgt.State[0], y = tgt.State[1], z = tgt.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);
            if (r > MaxRange) return null;
            double az = Math.Atan2(y, x);
            double el = Math.Atan2(z, Math.Sqrt(x * x + y * y));

            double nominalBW = BeamWidthRad;
            double effBW = nominalBW;
            const double maxEffBW = 30 * Math.PI / 180;
            if (r < referenceRange)
            {
                effBW = nominalBW * (referenceRange / r);
                if (effBW > maxEffBW) effBW = maxEffBW;
            }

            bool insideBeam;
            if (RadarType == "aircraft")
            {
                if (UseAesaMode)
                {
                    insideBeam = AesaBeams?.Any(b =>
                    {
                        double dAz = Math.Abs(MathUtil.NormalizeAngle(az - b.CurrentAzimuth));
                        if (dAz > effBW * 0.5) return false;
                        double dEl = Math.Abs(MathUtil.NormalizeAngle(el - b.CurrentElevation));
                        return dEl <= effBW; // wider vertical coverage in AESA
                    }) ?? false;
                }
                else
                {
                    double dAz = Math.Abs(MathUtil.NormalizeAngle(az - CurrentAzimuth));
                    if (dAz > effBW * 0.5) return null;
                    const double barHalfRad = Math.PI / 180; // 2° bar → 1° half‑width
                    double dEl = Math.Abs(MathUtil.NormalizeAngle(el - CurrentElevation));
                    insideBeam = dEl <= barHalfRad;
                }
            }
            else // ground radar
            {
                insideBeam = Math.Abs(MathUtil.NormalizeAngle(az - CurrentBeamAngle)) <= effBW * 0.5;
            }
            if (!insideBeam) return null;

            double snr_dB = ComputeAdvancedRadarEquationSNR(tgt.RCS, r);
            if (UseAesaMode) snr_dB += 3.0;
            if (snr_dB < requiredSNR_dB) return null;

            // Measurement noise scales with SNR: higher SNR → lower sigma.
            double snrLinForNoise = Math.Max(1e-6, Math.Pow(10.0, snr_dB / 10.0));
            double rangeSigma = rangeNoiseBase / Math.Sqrt(snrLinForNoise);
            double angleSigma = angleNoiseBase / Math.Sqrt(snrLinForNoise);

            double rMeas = r + Normal.Sample(rng, 0, rangeSigma);
            rMeas = Math.Max(1.0, rMeas);
            double azMeas = az + Normal.Sample(rng, 0, angleSigma);
            double elMeas = el + Normal.Sample(rng, 0, angleSigma);

            double snrLin = Math.Pow(10, snr_dB / 10.0) * (nominalBW / effBW);
            double amp = snrLin + Normal.Sample(rng, 0, 0.05 * snrLin);

            double radialVel = 0.0;
            if (UseDopplerProcessing && tgt.State.Count >= 6)
            {
                // State = [x, y, z, speed, headingRad, climbRate]
                double s = tgt.State[3];
                double h = tgt.State[4];
                double climb = tgt.State[5];

                // Convert speed/heading/climb to Cartesian velocity components
                double vx = s * Math.Cos(h);
                double vy = s * Math.Sin(h);
                double vz = climb;

                // Project velocity onto line-of-sight vector to get radial velocity
                radialVel = (x * vx + y * vy + z * vz) / (r + 1e-6) + Normal.Sample(rng, 0, VelocityNoiseStd);
            }

            return new Measurement
            {
                Range = rMeas,
                Azimuth = MathUtil.NormalizeAngle(azMeas),
                Elevation = MathUtil.NormalizeAngle(elMeas),
                Amplitude = amp,
                RadialVelocity = radialVel,
                SNR_dB = snr_dB
            };
        }

        private Measurement GenerateFalseAlarm()
        {
            double u = rng.NextDouble();
            double rFA = MaxRange * Math.Sqrt(u);
            double halfBW = BeamWidthRad * 0.5;
            double mainAz = RadarType == "aircraft" && UseAesaMode && AesaBeams?.Count > 0
                ? AesaBeams[rng.Next(AesaBeams.Count)].CurrentAzimuth
                : (RadarType == "aircraft" ? CurrentAzimuth : CurrentBeamAngle);
            double azFA = MathUtil.NormalizeAngle(mainAz + (rng.NextDouble() * BeamWidthRad - halfBW));
            double elCenter = RadarType == "aircraft" ? CurrentElevation : 0.0;
            double elFA = elCenter + Normal.Sample(rng, 0, angleNoiseBase * 2);

            double rMeas = Math.Max(1.0, rFA + Normal.Sample(rng, 0, rangeNoiseBase * 0.5));
            double azMeas = MathUtil.NormalizeAngle(azFA + Normal.Sample(rng, 0, angleNoiseBase));
            double elMeas = MathUtil.NormalizeAngle(elFA + Normal.Sample(rng, 0, angleNoiseBase));
            double baseAmp = Math.Pow(10.0, requiredSNR_dB / 10.0);
            double amp = Math.Max(0.8 * baseAmp, baseAmp + Normal.Sample(rng, 0, 0.2 * baseAmp));
            double snr_dB = requiredSNR_dB + Normal.Sample(rng, 0, 2.0);
            double vel = UseDopplerProcessing ? Normal.Sample(rng, 0, 100.0) : 0.0;

            return new Measurement { Range = rMeas, Azimuth = azMeas, Elevation = elMeas, Amplitude = amp, RadialVelocity = vel, SNR_dB = snr_dB };
        }

        // —————————————————————————  CFAR & clustering (deterministic)  —————————————————————————
        private List<Measurement> CFARFilterMeasurements(List<Measurement> meas)
        {
            if (meas.Count == 0) return meas;
            var sorted = meas.OrderBy(m => m.Range).ToArray();
            int n = sorted.Length;
            var results = new List<Measurement>(n);
            double[] amp = sorted.Select(m => m.Amplitude).ToArray();
            int left = 0, right = 0;
            for (int i = 0; i < n; i++)
            {
                double rCut = sorted[i].Range;
                while (right < n && sorted[right].Range - rCut <= CFARWindowWidth) right++;
                while (left < i && rCut - sorted[left].Range > CFARWindowWidth) left++;
                var noise = new List<double>();
                for (int k = left; k < right; k++)
                {
                    if (k == i) continue;
                    if (Math.Abs(sorted[k].Range - rCut) <= CFARGuardWidth) continue;
                    noise.Add(amp[k]);
                }
                double thresh = noise.Count == 0
                    ? 1.5 * Math.Pow(10, requiredSNR_dB / 10)
                    : CFARThresholdMultiplier * Percentile(noise, 0.75);
                if (amp[i] >= thresh) results.Add(sorted[i]);
            }
            return results;
        }

        private List<Measurement> DopplerCFARFilterMeasurements(List<Measurement> meas)
        {
            if (meas.Count == 0) return meas;
            var sorted = meas.OrderBy(m => m.RadialVelocity).ToArray();
            int n = sorted.Length;
            var results = new List<Measurement>(n);
            double[] amp = sorted.Select(m => m.Amplitude).ToArray();
            int left = 0, right = 0;
            for (int i = 0; i < n; i++)
            {
                double vCut = sorted[i].RadialVelocity;
                while (right < n && sorted[right].RadialVelocity - vCut <= DopplerCFARWindowWidth) right++;
                while (left < i && vCut - sorted[left].RadialVelocity > DopplerCFARWindowWidth) left++;
                var noise = new List<double>();
                for (int k = left; k < right; k++)
                {
                    if (k == i) continue;
                    if (Math.Abs(sorted[k].RadialVelocity - vCut) <= DopplerCFARGuardWidth) continue;
                    noise.Add(amp[k]);
                }
                double thresh = noise.Count == 0
                    ? 1.5 * Math.Pow(10, requiredSNR_dB / 10)
                    : DopplerCFARThresholdMultiplier * Percentile(noise, 0.75);
                if (amp[i] >= thresh) results.Add(sorted[i]);
            }
            return results;
        }

        private List<Measurement> MergeCloseDetections(List<Measurement> meas, double baseDist)
        {
            int n = meas.Count;
            if (n <= 1) return meas;
            int[] parent = Enumerable.Range(0, n).ToArray();
            int Find(int x) => parent[x] == x ? x : parent[x] = Find(parent[x]);
            void Union(int a, int b) { a = Find(a); b = Find(b); if (a != b) parent[b] = a; }
            for (int i = 0; i < n - 1; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    double rMax = Math.Max(meas[i].Range, meas[j].Range);
                    double angTol = 0.8 * BeamWidthRad * rMax;
                    double dCart = CartesianDistance(meas[i], meas[j]);
                    if (dCart < Math.Max(baseDist, angTol)) Union(i, j);
                }
            }
            var repr = new Dictionary<int, Measurement>();
            for (int i = 0; i < n; i++)
            {
                int root = Find(i);
                if (!repr.ContainsKey(root) || meas[i].Amplitude > repr[root].Amplitude)
                    repr[root] = meas[i];
            }
            return repr.Values.ToList();
        }

        // ————————————————————————————  small utility  ————————————————————————————
        private static double Percentile(List<double> data, double p)
        {
            data.Sort();
            if (data.Count == 0) return 0;
            double idx = p * (data.Count - 1);
            int lo = (int)Math.Floor(idx);
            int hi = (int)Math.Ceiling(idx);
            if (lo == hi) return data[lo];
            double w = idx - lo;
            return data[lo] * (1 - w) + data[hi] * w;
        }
    }
}
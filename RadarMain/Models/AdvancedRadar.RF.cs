using System;
using RealRadarSim.Utils;

namespace RealRadarSim.Models
{
    public partial class AdvancedRadar
    {
        /// <summary>Convenience shim for external classes.</summary>
        public double GetSNR_dB(double rcs, double range) => ComputeAdvancedRadarEquationSNR(rcs, range);

        // ———————————————————————————  radar‑equation / math only  ——————————————————————————
        private double ComputeAdvancedRadarEquationSNR(double rcs, double range)
        {
            if (UseReferenceSNRModel)
            {
                double otherLoss_dB = 2 * (AtmosphericLossOneWay_dB + WeatherLossOneWay_dB) + SystemLoss_dB;
                double rangeRatio_dB = 10 * Math.Log10(range / referenceRange);
                double rcsRatio_dB = 10 * Math.Log10(rcs / referenceRCS);
                return snr0_dB - (pathLossExponent_dB * rangeRatio_dB) + rcsRatio_dB - otherLoss_dB;
            }

            // — Full monostatic radar equation (simplified thermal‑noise model) —
            const double c = 3.0e8;
            const double k_dBW = -228.6; // 10·log10(k)
            const double T0_dB = 24.6;   // 10·log10(290 K)
            const double bandwidth_dB = 60.0; // 1 MHz IF bandwidth
            const double noiseFigure_dB = 5.0;
            const double pulseCount = 10.0;

            double lambda = c / FrequencyHz;
            double txPower_dBW = TxPower_dBm - 30.0;
            double totalGain_dB = 2 * AntennaGain_dBi;
            double rcs_dB = 10 * Math.Log10(rcs);
            double const_dB = 20 * Math.Log10(lambda) - 30 * Math.Log10(4 * Math.PI);
            double range_dB = -40 * Math.Log10(range);
            double losses_dB = 2 * (AtmosphericLossOneWay_dB + WeatherLossOneWay_dB) + SystemLoss_dB;
            double signal_dBW = txPower_dBW + totalGain_dB + const_dB + rcs_dB + range_dB - losses_dB;
            double noise_dBW = k_dBW + T0_dB + bandwidth_dB + noiseFigure_dB;
            double integration_dB = 10 * Math.Log10(pulseCount);
            return signal_dBW - noise_dBW + integration_dB;
        }

        // Cartesian helpers used by clustering
        private double CartesianDistance(Measurement a, Measurement b)
        {
            var (ax, ay, az) = ToCartesian(a);
            var (bx, by, bz) = ToCartesian(b);
            double dx = ax - bx, dy = ay - by, dz = az - bz;
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
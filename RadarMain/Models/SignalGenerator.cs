using System;
using System.Numerics;
using MathNet.Numerics.IntegralTransforms;

namespace RealRadarSim.Models
{
    public static class SignalGenerator
    {
        public static (float[] I, float[] Q) GenerateLfmChirp(double sampleRate, double duration, double bandwidth, double startFreq = 0.0)
        {
            int n = (int)Math.Round(sampleRate * duration);
            float[] i = new float[n];
            float[] q = new float[n];
            double k = bandwidth / duration;
            for (int idx = 0; idx < n; idx++)
            {
                double t = idx / sampleRate;
                double phase = 2 * Math.PI * (startFreq * t + 0.5 * k * t * t);
                i[idx] = (float)Math.Cos(phase);
                q[idx] = (float)Math.Sin(phase);
            }
            return (i, q);
        }

        public static double MeasureBandwidth(float[] i, float[] q, double sampleRate)
        {
            int n = i.Length;
            Complex32[] sig = new Complex32[n];
            for (int k = 0; k < n; k++)
                sig[k] = new Complex32(i[k], q[k]);
            Fourier.Forward(sig, FourierOptions.Matlab);
            double[] mag = new double[n];
            for (int k = 0; k < n; k++)
                mag[k] = sig[k].Magnitude;
            double max = 0;
            foreach (var m in mag) if (m > max) max = m;
            double threshold = max / Math.Sqrt(2.0); // -3 dB
            int left = 0, right = n - 1;
            while (left < n && mag[left] < threshold) left++;
            while (right >= 0 && mag[right] < threshold) right--;
            double bw = (right - left) * sampleRate / n;
            return bw;
        }
    }
}

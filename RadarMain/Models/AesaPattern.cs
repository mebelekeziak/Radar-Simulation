using System;
using MathNet.Numerics;
using System.Linq;
using MathNet.Numerics.IntegralTransforms;

namespace RealRadarSim.Models
{
    public enum AesaWindowFunction
    {
        None,
        Taylor,
        Chebyshev
    }

    public class AesaPatternOptions
    {
        public int ElementCount { get; set; } = 64;
        public double ElementSpacing { get; set; } = 0.5; // wavelengths
        public AesaWindowFunction Taper { get; set; } = AesaWindowFunction.None;
        public double SidelobeLevel_dB { get; set; } = 30.0;
        public int TaylorNBar { get; set; } = 4;
    }

    public static class AesaPattern
    {
        public static double[] ComputeWeights(AesaPatternOptions opt)
        {
            int N = opt.ElementCount;
            switch (opt.Taper)
            {
                case AesaWindowFunction.Taylor:
                    return TaylorWindow(N, opt.TaylorNBar, opt.SidelobeLevel_dB);
                case AesaWindowFunction.Chebyshev:
                    return ChebyshevWindow(N, opt.SidelobeLevel_dB);
                default:
                    return Enumerable.Repeat(1.0, N).ToArray();
            }
        }

        // Taylor window implementation based on scipy.signal.windows.taylor
        private static double[] TaylorWindow(int M, int nbar, double sll)
        {
            if (M <= 0) return Array.Empty<double>();
            double B = Math.Pow(10.0, sll / 20.0);
            double A = Math.Acosh(B) / Math.PI;
            double s2 = nbar * nbar / (A * A + Math.Pow(nbar - 0.5, 2));
            double[] ma = Enumerable.Range(1, nbar - 1).Select(x => (double)x).ToArray();
            double[] Fm = new double[nbar - 1];
            for (int mi = 0; mi < ma.Length; mi++)
            {
                double m = ma[mi];
                double numer = (mi % 2 == 0 ? 1.0 : -1.0);
                foreach (var mp in ma)
                {
                    numer *= 1 - (m * m) / (s2 * (A * A + Math.Pow(mp - 0.5, 2)));
                }
                double denom = 2.0;
                for (int k = 0; k < ma.Length; k++)
                {
                    if (k == mi) continue;
                    denom *= 1 - (m * m) / (ma[k] * ma[k]);
                }
                Fm[mi] = numer / denom;
            }
            double[] w = new double[M];
            for (int n = 0; n < M; n++)
            {
                double sum = 0.0;
                for (int mi = 0; mi < Fm.Length; mi++)
                {
                    double m = ma[mi];
                    sum += Fm[mi] * Math.Cos(2 * Math.PI * m * (n - M / 2.0 + 0.5) / M);
                }
                w[n] = 1.0 + 2.0 * sum;
            }
            // normalize
            double max = w[(M - 1) / 2];
            for (int i = 0; i < M; i++)
                w[i] /= max;
            return w;
        }

        // Dolph-Chebyshev window based on scipy.signal.windows.chebwin
        private static double[] ChebyshevWindow(int M, double at)
        {
            if (M <= 0) return Array.Empty<double>();
            double order = M - 1.0;
            double beta = Math.Cosh(Math.Acosh(Math.Pow(10.0, Math.Abs(at) / 20.0)) / order);
            Complex32[] p = new Complex32[M];
            for (int k = 0; k < M; k++)
            {
                double x = beta * Math.Cos(Math.PI * k / M);
                double val;
                if (x > 1)
                    val = Math.Cosh(order * Math.Acosh(x));
                else if (x < -1)
                    val = (2 * (M % 2) - 1) * Math.Cosh(order * Math.Acosh(-x));
                else
                    val = Math.Cos(order * Math.Acos(x));
                p[k] = new Complex32((float)val, 0f);
            }
            Complex32[] wComplex;
            if (M % 2 == 1)
            {
                Fourier.Forward(p, FourierOptions.Matlab);
                int n = (M + 1) / 2;
                wComplex = new Complex32[n];
                Array.Copy(p, 0, wComplex, 0, n);
                Array.Reverse(wComplex, 1, n - 1);
                wComplex = wComplex.Concat(p.Take(n)).ToArray();
            }
            else
            {
                for (int k = 0; k < M; k++)
                    p[k] *= Complex32.Exp(new Complex32(0f, (float)(Math.PI * k / M)));
                Fourier.Forward(p, FourierOptions.Matlab);
                int n = M / 2 + 1;
                wComplex = new Complex32[n];
                Array.Copy(p, 1, wComplex, 0, n - 1);
                Array.Reverse(wComplex, 0, n - 1);
                wComplex = wComplex.Concat(p.Skip(1).Take(n - 1)).ToArray();
            }
            double max = wComplex.Max(c => c.Real);
            return wComplex.Select(c => c.Real / max).ToArray();
        }

        public static double ArrayFactorGain(double angleRad, AesaPatternOptions opt)
        {
            double[] w = ComputeWeights(opt);
            int N = w.Length;
            double kd = 2 * Math.PI * opt.ElementSpacing;
            Complex32 sum = Complex32.Zero;
            for (int n = 0; n < N; n++)
            {
                double phase = kd * n * Math.Sin(angleRad);
                sum += w[n] * Complex32.Exp(new Complex32(0f, (float)phase));
            }
            return sum.Magnitude / w.Sum();
        }
    }
}

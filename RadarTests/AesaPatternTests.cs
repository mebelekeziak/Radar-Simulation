using Xunit;
using RealRadarSim.Models;
using System;
using System.Linq;

namespace RadarTests
{
    public class AesaPatternTests
    {
        [Fact]
        public void Taylor30dB_SidelobeBelow27dB()
        {
            var opts = new AesaPatternOptions
            {
                ElementCount = 64,
                Taper = AesaWindowFunction.Taylor,
                SidelobeLevel_dB = 30,
                TaylorNBar = 4
            };
            var w = AesaPattern.ComputeWeights(opts);
            int N = w.Length;
            double[] ang = Enumerable.Range(-500, 1001).Select(i => i * 0.001).ToArray();
            double peak = double.MinValue;
            double maxSll = double.MinValue;
            foreach (double a in ang)
            {
                double g = AesaPattern.ArrayFactorGain(a, opts);
                double db = 20 * Math.Log10(g);
                if (db > peak) peak = db;
            }
            foreach (double a in ang)
            {
                double g = AesaPattern.ArrayFactorGain(a, opts);
                double db = 20 * Math.Log10(g);
                if (Math.Abs(a) > 0.02) // exclude main lobe approx
                    if (db > maxSll) maxSll = db;
            }
            Assert.True(maxSll - peak <= -27.0);
        }
    }
}

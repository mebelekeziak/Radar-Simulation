using Xunit;
using RealRadarSim.Models;

namespace RadarTests
{
    public class ChirpTests
    {
        [Fact]
        public void ChirpBandwidthApprox40MHz()
        {
            double fs = 80e6; // sample rate
            var (i, q) = SignalGenerator.GenerateLfmChirp(fs, 10e-6, 40e6);
            double bw = SignalGenerator.MeasureBandwidth(i, q, fs);
            Assert.InRange(bw, 39.5e6, 40.5e6);
        }
    }
}

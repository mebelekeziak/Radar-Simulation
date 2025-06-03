using System;
using System.Collections.Generic;
using System.Linq;
using RealRadarSim.Models;

namespace RealRadarSim.Processing
{
    /// <summary>
    /// Simple pulse–Doppler processor that applies range gating and
    /// Doppler filtering to suppress clutter returns.
    /// </summary>
    public class PulseDopplerProcessor
    {
        public double MinRange { get; set; } = 500.0;
        public double MaxRange { get; set; } = 120000.0;
        public double ClutterVelocityThreshold { get; set; } = 10.0; // m/s

        /// <summary>
        /// Filter the raw measurements using range gates and a basic
        /// Doppler threshold. Measurements within the clutter velocity
        /// region are discarded.
        /// </summary>
        public List<Measurement> ProcessMeasurements(IEnumerable<Measurement> measurements)
        {
            return measurements
                .Where(m => m.Range >= MinRange && m.Range <= MaxRange)
                .Where(m => Math.Abs(m.RadialVelocity) >= ClutterVelocityThreshold)
                .ToList();
        }
    }
}

using System.Collections.Generic;
using System.Linq;
using RealRadarSim.Tracking;

namespace RealRadarSim.Models
{
    /// <summary>
    /// Basic AESA beam scheduler that assigns the strongest tracks to
    /// the available electronic beams each update cycle.
    /// </summary>
    public class AesaScheduler
    {
        private readonly AdvancedRadar _radar;

        public AesaScheduler(AdvancedRadar radar)
        {
            _radar = radar;
        }

        /// <summary>
        /// Assigns the top priority tracks to the radar's AESA beams.
        /// </summary>
        public void AssignBeams(IReadOnlyList<JPDA_Track> tracks)
        {
            if (_radar.AesaBeams == null)
                return;

            var ordered = tracks
                .Where(t => t.ExistenceProb > 0.5)
                .OrderByDescending(t => t.ExistenceProb)
                .Take(_radar.AesaBeams.Count)
                .ToList();

            for (int i = 0; i < _radar.AesaBeams.Count; i++)
            {
                if (i < ordered.Count)
                    _radar.AesaBeams[i].AssignTrack(ordered[i]);
                else
                    _radar.AesaBeams[i].ClearAssignment();
            }
        }
    }
}

namespace RealRadarSim.Models
{
    public interface IRadar
    {
        // Capabilities
        double MaxRange { get; }
        double BeamWidthRad { get; }
        string RadarType { get; }

        // Runtime status
        bool IsLocked { get; }
        double CurrentAzimuth { get; }

        // Main loop hooks
        void UpdateBeam(double dt);
        IReadOnlyList<Measurement> GetMeasurements(IReadOnlyList<TargetCT> targets);
    }
}

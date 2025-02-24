using System;
using System.Collections.Generic;
using System.IO;
using MoonSharp.Interpreter;
using MoonSharp.Interpreter.Loaders;
using RealRadarSim.Models;
using RealRadarSim.Tracking;
using RealRadarSim.Logging;

namespace RealRadarSim.Engine
{
    public class SimulationEngine
    {
        private double dt = 0.1;
        public double Time { get; private set; } = 0.0;

        private List<TargetCT> Targets = new List<TargetCT>();
        private AdvancedRadar Radar;
        private TrackManager trackManager;
        private Random rng;
        private List<Measurement> lastMeasurements = new List<Measurement>();

        // Throttle for target hit logging (in simulation seconds)
        private double lastTargetHitLogTime = -1.0;
        private const double targetHitLogThrottle = 1.0; // log at most once every 1.0 seconds

        public SimulationEngine(Random rng)
        {
            this.rng = rng;

            // Initialize the track manager with updated configuration.
            trackManager = new TrackManager(rng)
            {
                InitGateThreshold = 14.07,
                InitRequiredHits = 3,
                InitScanWindow = 3,
                InitPosStd = 200.0,
                InitVelStd = 100.0,
                // Remove GatingThreshold and use GatingProbability instead.
                GatingProbability = 0.99,
                AccelNoise = 2.0,
                ProbDetection = 0.9,
                ProbSurvival = 0.995,
                PruneThreshold = 0.01,
                MaxTrackMergeDist = 800.0,
                MaxTrackAge = 40.0,
                CandidateMergeDistance = 1500.0
            };

            if (File.Exists("config.lua"))
            {
                try
                {
                    Script script = new Script();
                    var loader = new FileSystemScriptLoader();
                    loader.ModulePaths = new string[] { "?.lua" };
                    script.Options.ScriptLoader = loader;
                    script.DoFile("config.lua");

                    DynValue configDyn = script.Globals.Get("config");
                    if (configDyn.Type == DataType.Table)
                    {
                        var config = configDyn.Table;
                        ConfigureFromLua(config);
                    }
                    else
                    {
                        InitializeDefaultConfiguration();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error loading config.lua: " + ex.Message);
                    InitializeDefaultConfiguration();
                }
            }
            else
            {
                InitializeDefaultConfiguration();
            }
        }

        private void ConfigureFromLua(Table config)
        {
            var radarDyn = config.Get("radar");
            if (radarDyn.Type == DataType.Table)
            {
                var radarTable = radarDyn.Table;

                double maxRange = GetNumberOrDefault(radarTable, "maxRange", 100000);
                double beamWidthDeg = GetNumberOrDefault(radarTable, "beamWidthDeg", 10.0);
                double rotationSpeedDegSec = GetNumberOrDefault(radarTable, "rotationSpeedDegSec", 36);
                double falseAlarmDensity = GetNumberOrDefault(radarTable, "falseAlarmDensity", 1e-10);

                // New SNR parameters (in dB)
                double snr0_dB = GetNumberOrDefault(radarTable, "snr0_dB", 30.0);
                double referenceRange = GetNumberOrDefault(radarTable, "referenceRange", 10000.0);
                double requiredSNR_dB = GetNumberOrDefault(radarTable, "requiredSNR_dB", 0.0);
                double rangeNoiseBase = GetNumberOrDefault(radarTable, "rangeNoiseBase", 100.0);
                double angleNoiseBase = GetNumberOrDefault(radarTable, "angleNoiseBase", 0.001);
                double cfarWindowWidth = GetNumberOrDefault(radarTable, "cfarWindowWidth", 5000.0);
                double cfarGuardWidth = GetNumberOrDefault(radarTable, "cfarGuardWidth", 300.0);
                double cfarThresholdMultiplier = GetNumberOrDefault(radarTable, "cfarThresholdMultiplier", 8.0);
                double clusterDistanceMeters = GetNumberOrDefault(radarTable, "clusterDistanceMeters", 600.0);
                string radarType = radarTable.Get("radarType").Type == DataType.String
                    ? radarTable.Get("radarType").String
                    : "ground";

                double antennaHeight = GetNumberOrDefault(radarTable, "antennaHeight", 0.0);
                bool showAzimuthBars = GetBoolOrDefault(radarTable, "showAzimuthBars", false);
                bool showElevationBars = GetBoolOrDefault(radarTable, "showElevationBars", false);
                double antennaAzimuthScan = GetNumberOrDefault(radarTable, "antennaAzimuthScan", 140.0);
                int antennaElevationBars = (int)GetNumberOrDefault(radarTable, "antennaElevationBars", 4);
                double tiltOffsetDeg = GetNumberOrDefault(radarTable, "tiltOffsetDeg", 0.0);

                // Lock-related configuration.
                double lockRange = GetNumberOrDefault(radarTable, "lockRange", 50000.0);
                double lockSNRThreshold_dB = GetNumberOrDefault(radarTable, "lockSNRThreshold_dB", 5.0);

                // Path loss exponent in dB.
                double pathLossExponent_dB = GetNumberOrDefault(radarTable, "pathLossExponent_dB", 40.0);

                bool useDopplerProcessing = GetBoolOrDefault(radarTable, "useDopplerProcessing", false);
                double velocityNoiseStd = GetNumberOrDefault(radarTable, "velocityNoiseStd", 1.0);
                bool useDopplerCFAR = GetBoolOrDefault(radarTable, "useDopplerCFAR", false);
                double dopplerCFARWindow = GetNumberOrDefault(radarTable, "dopplerCFARWindow", 150.0);
                double dopplerCFARGuard = GetNumberOrDefault(radarTable, "dopplerCFARGuard", 20.0);
                double dopplerCFARThresholdMultiplier = GetNumberOrDefault(radarTable, "dopplerCFARThresholdMultiplier", 6.0);

                Radar.UseDopplerProcessing = useDopplerProcessing;
                Radar.VelocityNoiseStd = velocityNoiseStd;
                Radar.UseDopplerCFAR = useDopplerCFAR;
                Radar.DopplerCFARWindow = dopplerCFARWindow;
                Radar.DopplerCFARGuard = dopplerCFARGuard;
                Radar.DopplerCFARThresholdMultiplier = dopplerCFARThresholdMultiplier;


                Radar = new AdvancedRadar(
                    maxRange,
                    beamWidthDeg,
                    rotationSpeedDegSec,
                    falseAlarmDensity,
                    snr0_dB,
                    referenceRange,
                    requiredSNR_dB,
                    rangeNoiseBase,
                    angleNoiseBase,
                    rng,
                    cfarWindowWidth,
                    cfarGuardWidth,
                    cfarThresholdMultiplier,
                    clusterDistanceMeters,
                    radarType.ToLower(),
                    antennaElevationBars,
                    antennaAzimuthScan,
                    tiltOffsetDeg,
                    lockRange,
                    lockSNRThreshold_dB,
                    pathLossExponent_dB
                );

                Radar.ShowAzimuthBars = showAzimuthBars;
                Radar.ShowElevationBars = showElevationBars;

                if (radarType.ToLower() == "aircraft")
                {
                    string opMode = radarTable.Get("operationMode").Type == DataType.String
                        ? radarTable.Get("operationMode").String.ToLower()
                        : "mechanical";
                    Radar.UseAesaMode = (opMode == "aesa");
                }

            }
            else
            {
                InitializeDefaultConfiguration();
            }

            // Target configuration.
            var targetsDyn = config.Get("targets");
            if (targetsDyn.Type == DataType.Table)
            {
                var targetsTable = targetsDyn.Table;
                var newTargets = new List<TargetCT>();

                foreach (var pair in targetsTable.Pairs)
                {
                    if (pair.Value.Type == DataType.Table)
                    {
                        var tt = pair.Value.Table;
                        double x = GetNumberOrDefault(tt, "x", 0);
                        double y = GetNumberOrDefault(tt, "y", 0);
                        double z = GetNumberOrDefault(tt, "z", 0);
                        double speed = GetNumberOrDefault(tt, "speed", 0);
                        double headingDeg = GetNumberOrDefault(tt, "headingDeg", 0);
                        double climbRate = GetNumberOrDefault(tt, "climbRate", 0);
                        double turnRateDeg = GetNumberOrDefault(tt, "turnRateDeg", 0);
                        double pStd = GetNumberOrDefault(tt, "processStd", 0.5);
                        string acName = tt.Get("aircraftName").Type == DataType.String
                            ? tt.Get("aircraftName").String
                            : "Unknown";
                        double rcs = GetNumberOrDefault(tt, "rcs", 10);

                        newTargets.Add(new TargetCT(
                            x, y, z,
                            speed, headingDeg, climbRate,
                            turnRateDeg, pStd,
                            acName, rcs,
                            rng
                        ));
                    }
                }

                if (newTargets.Count > 0)
                {
                    Targets = newTargets;
                }
                else
                {
                    SetDefaultTargets();
                }
            }
            else
            {
                SetDefaultTargets();
            }

            // Track Manager configuration.
            var trackManagerDyn = config.Get("trackManager");
            if (trackManagerDyn.Type == DataType.Table)
            {
                var tm = trackManagerDyn.Table;
                trackManager.InitGateThreshold = GetNumberOrDefault(tm, "initGateThreshold", 14.07);
                trackManager.InitRequiredHits = (int)GetNumberOrDefault(tm, "initRequiredHits", 3);
                trackManager.InitScanWindow = (int)GetNumberOrDefault(tm, "initScanWindow", 3);
                trackManager.InitPosStd = GetNumberOrDefault(tm, "initPosStd", 200.0);
                trackManager.InitVelStd = GetNumberOrDefault(tm, "initVelStd", 100.0);
                // Use gatingProbability instead of gatingThreshold.
                trackManager.GatingProbability = GetNumberOrDefault(tm, "gatingProbability", 0.99);
                trackManager.AccelNoise = GetNumberOrDefault(tm, "accelNoise", 2.0);
                trackManager.ProbDetection = GetNumberOrDefault(tm, "probDetection", 0.9);
                trackManager.ProbSurvival = GetNumberOrDefault(tm, "probSurvival", 0.995);
                trackManager.PruneThreshold = GetNumberOrDefault(tm, "pruneThreshold", 0.01);
                trackManager.MaxTrackMergeDist = GetNumberOrDefault(tm, "maxTrackMergeDist", 800.0);
                trackManager.MaxTrackAge = GetNumberOrDefault(tm, "maxTrackAge", 40.0);
                trackManager.CandidateMergeDistance = GetNumberOrDefault(tm, "candidateMergeDistance", 1500.0);
            }
        }

        private void InitializeDefaultConfiguration()
        {
            Radar = new AdvancedRadar(
                100000, 10.0, 36.0, 1e-10,
                30.0,       // snr0_dB
                10000.0,
                0.0,        // requiredSNR_dB
                100.0,
                0.001,
                rng,
                5000.0, 300.0, 8.0, 600.0,
                "ground",
                1,
                140.0,
                0.0,
                50000.0,
                5.0,
                40.0       // pathLossExponent_dB
            );
            SetDefaultTargets();
        }

        private void SetDefaultTargets()
        {
            Targets = new List<TargetCT>
            {
                new TargetCT(20000, 15000, 2000, 180, 45,   5, 2, 0.5, "Boeing 737",  40.0, rng),
                new TargetCT(-25000, -20000, 1000, 220, -60, 0, 3, 0.5, "Airbus A320", 30.0, rng),
                new TargetCT(10000, -10000, 3000, 120, 90, -5, 1, 0.3,  "Boeing 777",  60.0, rng),
                new TargetCT(60000, -45000, 3000, 270, 90, 15, 1, 0.3,  "F-16",       25.0, rng)
            };
        }

        private double GetNumberOrDefault(Table tbl, string key, double defVal)
        {
            var dynVal = tbl.Get(key);
            return dynVal.Type == DataType.Number ? dynVal.Number : defVal;
        }

        private bool GetBoolOrDefault(Table tbl, string key, bool defVal)
        {
            var dynVal = tbl.Get(key);
            if (dynVal.Type == DataType.Boolean)
                return dynVal.Boolean;
            else if (dynVal.Type == DataType.Number)
                return Math.Abs(dynVal.Number) > double.Epsilon;
            return defVal;
        }

        public void Update()
        {
            Time += dt;

            // Update target positions.
            foreach (var tgt in Targets)
                tgt.Update(dt);

            // Update radar beam.
            Radar.UpdateBeam(dt);

            // Generate measurements.
            var measurements = Radar.GetMeasurements(Targets);
            lastMeasurements = measurements;

            // --- Debug logging: Log only if at least one target measurement is generated.
            // We assume that target-generated measurements have Amplitude > 10
            int targetMeasurementCount = 0;
            foreach (var m in measurements)
            {
                if (m.Amplitude > 10)
                    targetMeasurementCount++;
            }
            if (targetMeasurementCount > 0 && (Time - lastTargetHitLogTime) >= targetHitLogThrottle)
            {
                DebugLogger.LogMeasurement($"Radar beam hit target: {targetMeasurementCount} target measurement(s) at simulation time {Time:F2}s.");
                lastTargetHitLogTime = Time;
            }

            // Update tracks.
            trackManager.UpdateTracks(measurements, dt);

            // Optional: assign flight names if tracks are near a known target.
            var tracks = trackManager.GetTracks();
            foreach (var trk in tracks)
            {
                if (trk.ExistenceProb < 0.20) continue;
                if (string.IsNullOrEmpty(trk.FlightName))
                {
                    double bestDist = double.MaxValue;
                    TargetCT bestTgt = null;
                    foreach (var t in Targets)
                    {
                        double dx = trk.Filter.State[0] - t.State[0];
                        double dy = trk.Filter.State[1] - t.State[1];
                        double dz = trk.Filter.State[2] - t.State[2];
                        double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestTgt = t;
                        }
                    }
                    if (bestTgt != null && bestDist < 5000)
                        trk.FlightName = bestTgt.AircraftName;
                }
            }
        }

        public double GetDt() => dt;
        public double GetCurrentBeamAngle()
        {
            return (Radar.RadarType.ToLower() == "aircraft") ? Radar.CurrentAzimuth : Radar.CurrentBeamAngle;
        }
        public double GetMaxRange() => Radar.MaxRange;
        public List<TargetCT> GetTargets() => Targets;
        public List<JPDA_Track> GetTracks() => trackManager.GetTracks();
        public List<Measurement> GetLastMeasurements() => lastMeasurements;
        public AdvancedRadar GetRadar() => Radar;
        public string GetRadarType() => Radar.RadarType;
    }
}
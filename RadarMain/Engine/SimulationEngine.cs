using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using MoonSharp.Interpreter;
using MoonSharp.Interpreter.Loaders;
using RealRadarSim.Models;
using RealRadarSim.Tracking;
using RealRadarSim.Logging;
using RealRadarSim.Utils;

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
        private readonly List<string> configWarnings = new List<string>();

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
                        // Call a modified configuration routine that also outputs debug info
                        ConfigureFromLua(config);
                    }
                    else
                    {
                        InitializeDefaultConfiguration();
                    }
                }
                catch (Exception ex)
                {
                    RealRadarSim.Logging.DebugLogger.Log("Lua", $"Error loading config.lua: {ex.Message}");
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
                string radarTypeRaw = radarTable.Get("radarType").Type == DataType.String
                    ? radarTable.Get("radarType").String
                    : "ground";
                string radarType = (radarTypeRaw ?? "ground").ToLower();
                if (radarType != "ground" && radarType != "aircraft")
                {
                    configWarnings.Add($"Unsupported radar.radarType='{radarTypeRaw}', defaulting to 'ground'.");
                    radarType = "ground";
                }

                string antennaPatternRaw = radarTable.Get("antennaPattern").Type == DataType.String
                    ? radarTable.Get("antennaPattern").String
                    : "gaussian";
                string antennaPattern = (antennaPatternRaw ?? "gaussian").ToLower();
                bool antennaPatternOk = antennaPattern == "gaussian" || antennaPattern == "sinc2" || antennaPattern == "sinc^2" || antennaPattern == "sinc";
                if (!antennaPatternOk)
                {
                    configWarnings.Add($"Unsupported radar.antennaPattern='{antennaPatternRaw}', defaulting to 'gaussian'.");
                    antennaPattern = "gaussian";
                }

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
                double FrequencyHz = GetNumberOrDefault(radarTable, "FrequencyHz", 3e9);
                double TxPower_dBm = GetNumberOrDefault(radarTable, "TxPower_dBm", 70.0);
                double AntennaGain_dBi = GetNumberOrDefault(radarTable, "AntennaGain_dBi", 101.0);

                // Instantiate Radar first
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
                    pathLossExponent_dB,
                    FrequencyHz,
                    TxPower_dBm,
                    AntennaGain_dBi
                );

                // Now set Doppler processing properties on the instantiated Radar
                Radar.UseDopplerProcessing = useDopplerProcessing;
                Radar.VelocityNoiseStd = velocityNoiseStd;
                Radar.UseDopplerCFAR = useDopplerCFAR;
                Radar.DopplerCFARWindow = dopplerCFARWindow;
                Radar.DopplerCFARGuard = dopplerCFARGuard;
                Radar.DopplerCFARThresholdMultiplier = dopplerCFARThresholdMultiplier;

                // Mirror Doppler settings into TrackManager for fusion
                trackManager.UseDopplerProcessing = useDopplerProcessing;
                trackManager.VelocityNoiseStd = velocityNoiseStd;
                // Also pass measurement noise bases so EKF baseline/gating aligns with generator
                trackManager.RangeNoiseBase = rangeNoiseBase;
                trackManager.AngleNoiseBase = angleNoiseBase;

                // Auto-scale persistence to revisit time (general fix for slow scans)
                // Ground radar revisits every 360°; aircraft sweeps left-right across its azimuth span.
                double rotSpeed = MathUtil.DegToRad(rotationSpeedDegSec); // rad/s
                double revisitSec;
                if ((radarType == "aircraft") && antennaAzimuthScan > 0.0)
                {
                    // Approximate: traverse from -A/2 to +A/2 and back → 2*A degrees
                    revisitSec = MathUtil.DegToRad(2.0 * antennaAzimuthScan) / Math.Max(1e-6, rotSpeed);
                }
                else
                {
                    revisitSec = (2.0 * Math.PI) / Math.Max(1e-6, rotSpeed);
                }

                // If user provided an unrealistically small MaxTrackAge (e.g., < 4 sweeps),
                // clamp it up to avoid premature pruning just before the next revisit.
                // Using 4× revisit covers three consecutive misses plus a margin before the 4th pass.
                double minAge = 4.0 * revisitSec;
                if (trackManager.MaxTrackAge < minAge)
                {
                    RealRadarSim.Logging.DebugLogger.Log("Config", $"MaxTrackAge increased to {minAge:F1}s (>=4× revisit {revisitSec:F1}s).");
                    trackManager.MaxTrackAge = minAge;
                }

                // Use an existence half-life of ~3 sweeps so P(exist) decays smoothly across misses
                // and remains high enough for association until the next pass.
                trackManager.ExistenceHalfLifeSec = 3.0 * revisitSec;

                string debugText = $"[DEBUG] Loaded radar config values:\n" +
                                   $"FrequencyHz: {FrequencyHz}\n" +
                                   $"TxPower_dBm: {TxPower_dBm}\n" +
                                   $"AntennaGain_dBi: {AntennaGain_dBi}\n";
                RealRadarSim.Logging.DebugLogger.Log("Lua", debugText);

                Radar.ShowAzimuthBars = showAzimuthBars;
                Radar.ShowElevationBars = showElevationBars;
                Radar.AntennaPattern = antennaPattern;

                if (radarType == "aircraft")
                {
                    string opModeRaw = radarTable.Get("operationMode").Type == DataType.String
                        ? radarTable.Get("operationMode").String
                        : "mechanical";
                    string opMode = (opModeRaw ?? "mechanical").ToLower();
                    if (opMode != "aesa" && opMode != "mechanical")
                    {
                        configWarnings.Add($"Unsupported radar.operationMode='{opModeRaw}', defaulting to 'mechanical'.");
                        opMode = "mechanical";
                    }
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

                // If the Lua config explicitly sets a higher MaxTrackAge, keep it;
                // otherwise (or if it's too small) it may have been adjusted above.
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
            // Default Doppler disabled; keep TrackManager consistent
            trackManager.UseDopplerProcessing = Radar.UseDopplerProcessing;
            trackManager.VelocityNoiseStd = Radar.VelocityNoiseStd;
            // Align TrackManager noise bases with defaults used in AdvancedRadar above
            trackManager.RangeNoiseBase = 100.0;
            trackManager.AngleNoiseBase = 0.001;

            // Scale age/decay with revisit time for defaults too
            double rotSpeed = MathUtil.DegToRad(36.0); // rad/s from default above
            double revisitSec = (2.0 * Math.PI) / Math.Max(1e-6, rotSpeed);
            trackManager.MaxTrackAge = Math.Max(trackManager.MaxTrackAge, 4.0 * revisitSec);
            trackManager.ExistenceHalfLifeSec = 3.0 * revisitSec;
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

            // Optimized flight-name assignment using KD‑tree, squared distances, and parallelization
            AssignFlightNamesToTracks();
        }

        private void AssignFlightNamesToTracks()
        {
            // Build KD‑tree on current Targets
            var kdTree = KdTreeNode.Build(Targets, 0);
            var tracks = trackManager.GetTracks();
            double thresholdSq = 5000.0 * 5000.0;

            Parallel.ForEach(tracks, trk =>
            {
                if (trk.ExistenceProb < 0.20) return;
                if (!string.IsNullOrEmpty(trk.FlightName)) return;

                var point = new double[]
                {
                    trk.Filter.State[0],
                    trk.Filter.State[1],
                    trk.Filter.State[2]
                };

                TargetCT nearest = null;
                double bestDistSq = double.MaxValue;
                KdTreeNode.Nearest(kdTree, point, ref nearest, ref bestDistSq);

                if (nearest != null && bestDistSq < thresholdSq)
                {
                    trk.FlightName = nearest.AircraftName;
                }
            });
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
        public IReadOnlyList<string> GetConfigWarnings() => configWarnings.AsReadOnly();

        // --- KD‑tree implementation for 3D nearest-neighbor search ---
        private class KdTreeNode
        {
            public TargetCT Target;
            public int Axis;
            public KdTreeNode Left;
            public KdTreeNode Right;

            private KdTreeNode(TargetCT target, int axis)
            {
                Target = target;
                Axis = axis;
            }

            public static KdTreeNode Build(List<TargetCT> targets, int depth)
            {
                if (targets == null || targets.Count == 0) return null;
                int axis = depth % 3;
                targets.Sort((a, b) =>
                {
                    double aCoord = a.State[axis];
                    double bCoord = b.State[axis];
                    return aCoord.CompareTo(bCoord);
                });
                int median = targets.Count / 2;
                var node = new KdTreeNode(targets[median], axis);
                var leftList = targets.GetRange(0, median);
                var rightList = targets.GetRange(median + 1, targets.Count - median - 1);
                node.Left = Build(leftList, depth + 1);
                node.Right = Build(rightList, depth + 1);
                return node;
            }

            public static void Nearest(KdTreeNode node, double[] point, ref TargetCT best, ref double bestDistSq)
            {
                if (node == null) return;

                var coords = node.Target.State;
                double dx = point[0] - coords[0];
                double dy = point[1] - coords[1];
                double dz = point[2] - coords[2];
                double distSq = dx * dx + dy * dy + dz * dz;

                if (distSq < bestDistSq)
                {
                    bestDistSq = distSq;
                    best = node.Target;
                }

                int axis = node.Axis;
                double diff = point[axis] - coords[axis];
                var first = diff < 0 ? node.Left : node.Right;
                var second = diff < 0 ? node.Right : node.Left;

                Nearest(first, point, ref best, ref bestDistSq);
                if (diff * diff < bestDistSq)
                {
                    Nearest(second, point, ref best, ref bestDistSq);
                }
            }
        }
    }
}

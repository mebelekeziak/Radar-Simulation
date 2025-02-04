using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Distributions;
using MoonSharp.Interpreter;
using MoonSharp.Interpreter.Loaders;
using RealRadarSim.Models;
using RealRadarSim.Tracking;
using System.Diagnostics.Metrics;

namespace RealRadarSim.Engine
{
    public class SimulationEngine
    {
        private double dt = 0.1; // Update interval in seconds.
        public double Time { get; private set; } = 0.0;
        private List<TargetCT> Targets;
        private AdvancedRadar Radar;
        private TrackManager trackManager;
        private Random rng;

        // Store the last set of raw measurements for debugging.
        private List<Measurement> lastMeasurements = new List<Measurement>();

        public SimulationEngine(Random rng)
        {
            this.rng = rng;
            trackManager = new TrackManager(rng)
            {
                // Default track parameters (can be overridden by Lua)
                InitGateThreshold = 14.07,
                InitRequiredHits = 3,
                InitScanWindow = 3,
                InitPosStd = 200.0,
                InitVelStd = 100.0,
                GatingThreshold = 14.07,
                AccelNoise = 2.0,
                ProbDetection = 0.9,
                ProbSurvival = 0.995,
                PruneThreshold = 0.01,
                MaxTrackMergeDist = 800.0,
                MaxTrackAge = 20.0,
                CandidateMergeDistance = 1500.0
            };

            // Load configuration from Lua if available.
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
                        // Radar configuration
                        var radarDyn = config.Get("radar");
                        if (radarDyn.Type == DataType.Table)
                        {
                            var radarTable = radarDyn.Table;
                            double maxRange = GetNumberOrDefault(radarTable, "maxRange", 100000);
                            double beamWidthDeg = GetNumberOrDefault(radarTable, "beamWidthDeg", 10.0);
                            double rotationSpeedDegSec = GetNumberOrDefault(radarTable, "rotationSpeedDegSec", 36);
                            double falseAlarmDensity = GetNumberOrDefault(radarTable, "falseAlarmDensity", 1e-10);
                            double snr0 = GetNumberOrDefault(radarTable, "snr0", 15000.0);
                            double referenceRange = GetNumberOrDefault(radarTable, "referenceRange", 10000.0);
                            double requiredSNR = GetNumberOrDefault(radarTable, "requiredSNR", 0.2);
                            double rangeNoiseBase = GetNumberOrDefault(radarTable, "rangeNoiseBase", 100.0);
                            double angleNoiseBase = GetNumberOrDefault(radarTable, "angleNoiseBase", 0.001);
                            // CFAR parameters
                            double cfarWindowWidth = GetNumberOrDefault(radarTable, "cfarWindowWidth", 5000.0);
                            double cfarGuardWidth = GetNumberOrDefault(radarTable, "cfarGuardWidth", 300.0);
                            double cfarThresholdMultiplier = GetNumberOrDefault(radarTable, "cfarThresholdMultiplier", 8.0);
                            double clusterDistanceMeters = GetNumberOrDefault(radarTable, "clusterDistanceMeters", 600.0);

                            Radar = new AdvancedRadar(
                                maxRange, beamWidthDeg, rotationSpeedDegSec,
                                falseAlarmDensity, snr0, referenceRange,
                                requiredSNR, rangeNoiseBase, angleNoiseBase, rng,
                                cfarWindowWidth, cfarGuardWidth, cfarThresholdMultiplier, clusterDistanceMeters);
                        }
                        else
                        {
                            Radar = new AdvancedRadar(100000, 10.0, 36, 1e-10, 15000.0,
                                10000.0, 0.2, 100.0, 0.001, rng);
                        }

                        // Target configuration
                        var targetsDyn = config.Get("targets");
                        if (targetsDyn.Type == DataType.Table)
                        {
                            var targetsTable = targetsDyn.Table;
                            Targets = new List<TargetCT>();
                            foreach (var pair in targetsTable.Pairs)
                            {
                                if (pair.Value.Type == DataType.Table)
                                {
                                    var targetTable = pair.Value.Table;
                                    double x = GetNumberOrDefault(targetTable, "x", 0);
                                    double y = GetNumberOrDefault(targetTable, "y", 0);
                                    double z = GetNumberOrDefault(targetTable, "z", 0);
                                    double speed = GetNumberOrDefault(targetTable, "speed", 0);
                                    double headingDeg = GetNumberOrDefault(targetTable, "headingDeg", 0);
                                    double climbRate = GetNumberOrDefault(targetTable, "climbRate", 0);
                                    double turnRateDeg = GetNumberOrDefault(targetTable, "turnRateDeg", 0);
                                    double processStd = GetNumberOrDefault(targetTable, "processStd", 0.5);
                                    string aircraftName = targetTable.Get("aircraftName").Type == DataType.String
                                                            ? targetTable.Get("aircraftName").String
                                                            : "Unknown";
                                    double rcs = GetNumberOrDefault(targetTable, "rcs", 10.0);

                                    Targets.Add(new TargetCT(x, y, z, speed, headingDeg,
                                        climbRate, turnRateDeg, processStd, aircraftName, rcs, rng));
                                }
                            }
                            if (Targets.Count == 0)
                                SetDefaultTargets();
                        }
                        else
                        {
                            SetDefaultTargets();
                        }

                        // Optionally load trackManager parameters from Lua config.
                        var trackManagerDyn = config.Get("trackManager");
                        if (trackManagerDyn.Type == DataType.Table)
                        {
                            var trackTable = trackManagerDyn.Table;
                            trackManager.InitGateThreshold = GetNumberOrDefault(trackTable, "initGateThreshold", 14.07);
                            trackManager.InitRequiredHits = (int)GetNumberOrDefault(trackTable, "initRequiredHits", 3);
                            trackManager.InitScanWindow = (int)GetNumberOrDefault(trackTable, "initScanWindow", 3);
                            trackManager.InitPosStd = GetNumberOrDefault(trackTable, "initPosStd", 200.0);
                            trackManager.InitVelStd = GetNumberOrDefault(trackTable, "initVelStd", 100.0);
                            trackManager.GatingThreshold = GetNumberOrDefault(trackTable, "gatingThreshold", 14.07);
                            trackManager.AccelNoise = GetNumberOrDefault(trackTable, "accelNoise", 2.0);
                            trackManager.ProbDetection = GetNumberOrDefault(trackTable, "probDetection", 0.9);
                            trackManager.ProbSurvival = GetNumberOrDefault(trackTable, "probSurvival", 0.995);
                            trackManager.PruneThreshold = GetNumberOrDefault(trackTable, "pruneThreshold", 0.01);
                            trackManager.MaxTrackMergeDist = GetNumberOrDefault(trackTable, "maxTrackMergeDist", 800.0);
                            trackManager.MaxTrackAge = GetNumberOrDefault(trackTable, "maxTrackAge", 20.0);
                            trackManager.CandidateMergeDistance = GetNumberOrDefault(trackTable, "candidateMergeDistance", 1500.0);
                        }
                    }
                    else
                    {
                        SetDefaultConfiguration();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error loading Lua config: " + ex.Message);
                    SetDefaultConfiguration();
                }
            }
            else
            {
                SetDefaultConfiguration();
            }
        }

        private void SetDefaultConfiguration()
        {
            SetDefaultTargets();
            Radar = new AdvancedRadar(100000, 10.0, 36, 1e-10, 15000.0,
                10000.0, 0.2, 100.0, 0.001, rng);
        }

        private void SetDefaultTargets()
        {
            Targets = new List<TargetCT>
            {
                new TargetCT(20000, 15000, 2000, 180, 45, 5, 2, 0.5, "Boeing 737", 40.0, rng),
                new TargetCT(-25000, -20000, 1000, 220, -60, 0, 3, 0.5, "Airbus A320", 30.0, rng),
                new TargetCT(10000, -10000, 3000, 120, 90, -5, 1, 0.3, "Boeing 777", 60.0, rng),
                new TargetCT(60000, -45000, 3000, 120, 90, -5, 1, 0.3, "C-130", 35.0, rng)
            };
        }

        private double GetNumberOrDefault(Table table, string key, double defaultValue)
        {
            var val = table.Get(key);
            return val.Type == DataType.Number ? val.Number : defaultValue;
        }

        public void Update()
        {
            Time += dt;
            foreach (var tgt in Targets)
                tgt.Update(dt);

            Radar.UpdateBeam(dt);

            // Get and store raw measurements (for debugging)
            var measurements = Radar.GetMeasurements(Targets);
            lastMeasurements = measurements;

            trackManager.UpdateTracks(measurements, dt);

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
        public double GetCurrentBeamAngle() => Radar.CurrentBeamAngle;
        public double GetMaxRange() => Radar.MaxRange;
        public List<TargetCT> GetTargets() => Targets;
        public List<JPDA_Track> GetTracks() => trackManager.GetTracks();
        public List<Measurement> GetLastMeasurements() => lastMeasurements;
    }
}

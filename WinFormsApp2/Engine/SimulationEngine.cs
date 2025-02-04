﻿using System;
using System.Collections.Generic;
using System.IO;
using MoonSharp.Interpreter;
using MoonSharp.Interpreter.Loaders;
using RealRadarSim.Models;
using RealRadarSim.Tracking;

namespace RealRadarSim.Engine
{
    public class SimulationEngine
    {
        private double dt = 0.1;
        public double Time { get; private set; } = 0.0;

        // Non-null after constructor finishes
        private List<TargetCT> Targets = new List<TargetCT>();
        private AdvancedRadar Radar;
        private TrackManager trackManager;
        private Random rng;
        private List<Measurement> lastMeasurements = new List<Measurement>();

        public SimulationEngine(Random rng)
        {
            this.rng = rng;

            // Simple track manager (assuming you have a "TrackManager" in RealRadarSim.Tracking)
            trackManager = new TrackManager(rng)
            {
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

            // Attempt to load config.lua
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
            // Radar config
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
                double cfarWindowWidth = GetNumberOrDefault(radarTable, "cfarWindowWidth", 5000.0);
                double cfarGuardWidth = GetNumberOrDefault(radarTable, "cfarGuardWidth", 300.0);
                double cfarThresholdMultiplier = GetNumberOrDefault(radarTable, "cfarThresholdMultiplier", 8.0);
                double clusterDistanceMeters = GetNumberOrDefault(radarTable, "clusterDistanceMeters", 600.0);
                // Radar type:
                string radarType = radarTable.Get("radarType").Type == DataType.String
                    ? radarTable.Get("radarType").String
                    : "ground";

                // Additional booleans:
                double antennaHeight = GetNumberOrDefault(radarTable, "antennaHeight", 0.0);
                bool showAzimuthBars = GetBoolOrDefault(radarTable, "showAzimuthBars", false);
                bool showElevationBars = GetBoolOrDefault(radarTable, "showElevationBars", false);

                // Horizontal and bar config:
                double antennaAzimuthScan = GetNumberOrDefault(radarTable, "antennaAzimuthScan", 140.0);
                int antennaElevationBars = (int)GetNumberOrDefault(radarTable, "antennaElevationBars", 4);
                double barSpacing = GetNumberOrDefault(radarTable, "barSpacingDeg", 2.0);

                // NEW: read tilt offset from Lua
                double tiltOffsetDeg = GetNumberOrDefault(radarTable, "tiltOffsetDeg", 0.0);

                // Construct radar
                Radar = new AdvancedRadar(
                    maxRange,
                    beamWidthDeg,
                    rotationSpeedDegSec,
                    falseAlarmDensity,
                    snr0,
                    referenceRange,
                    requiredSNR,
                    rangeNoiseBase,
                    angleNoiseBase,
                    rng,
                    cfarWindowWidth,
                    cfarGuardWidth,
                    cfarThresholdMultiplier,
                    clusterDistanceMeters,
                    radarType.ToLower(),
                    antennaElevationBars,  // use antennaElevationBars as the int antenna height
                    antennaAzimuthScan,    // antenna azimuth scan in degrees
                    tiltOffsetDeg          // tilt offset in degrees
                );

                // Assign additional properties: // if you need to override the antenna height from Lua
                Radar.ShowAzimuthBars = showAzimuthBars;
                Radar.ShowElevationBars = showElevationBars;
            }
            else
            {
                InitializeDefaultConfiguration();
            }

            // Targets
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

            // track manager config
            var trackManagerDyn = config.Get("trackManager");
            if (trackManagerDyn.Type == DataType.Table)
            {
                var tm = trackManagerDyn.Table;
                trackManager.InitGateThreshold = GetNumberOrDefault(tm, "initGateThreshold", 14.07);
                trackManager.InitRequiredHits = (int)GetNumberOrDefault(tm, "initRequiredHits", 3);
                trackManager.InitScanWindow = (int)GetNumberOrDefault(tm, "initScanWindow", 3);
                trackManager.InitPosStd = GetNumberOrDefault(tm, "initPosStd", 200.0);
                trackManager.InitVelStd = GetNumberOrDefault(tm, "initVelStd", 100.0);
                trackManager.GatingThreshold = GetNumberOrDefault(tm, "gatingThreshold", 14.07);
                trackManager.AccelNoise = GetNumberOrDefault(tm, "accelNoise", 2.0);
                trackManager.ProbDetection = GetNumberOrDefault(tm, "probDetection", 0.9);
                trackManager.ProbSurvival = GetNumberOrDefault(tm, "probSurvival", 0.995);
                trackManager.PruneThreshold = GetNumberOrDefault(tm, "pruneThreshold", 0.01);
                trackManager.MaxTrackMergeDist = GetNumberOrDefault(tm, "maxTrackMergeDist", 800.0);
                trackManager.MaxTrackAge = GetNumberOrDefault(tm, "maxTrackAge", 20.0);
                trackManager.CandidateMergeDistance = GetNumberOrDefault(tm, "candidateMergeDistance", 1500.0);
            }
        }

        private void InitializeDefaultConfiguration()
        {
            Radar = new AdvancedRadar(
                100000, 10.0, 36.0, 1e-10,
                15000.0, 10000.0, 0.2, 100.0, 0.001,
                rng
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

        // ADD THIS METHOD to fix the missing GetBoolOrDefault error.
        private bool GetBoolOrDefault(Table tbl, string key, bool defVal)
        {
            var dynVal = tbl.Get(key);
            if (dynVal.Type == DataType.Boolean)
            {
                return dynVal.Boolean;
            }
            else if (dynVal.Type == DataType.Number)
            {
                // treat any nonzero number as 'true'
                return Math.Abs(dynVal.Number) > double.Epsilon;
            }
            return defVal;
        }

        public void Update()
        {
            Time += dt;

            // move targets
            foreach (var tgt in Targets)
                tgt.Update(dt);

            // radar scanning
            Radar.UpdateBeam(dt);

            // measurements
            var measurements = Radar.GetMeasurements(Targets);
            lastMeasurements = measurements;

            // track manager
            trackManager.UpdateTracks(measurements, dt);

            // optional logic: label track with flight name if near a known target
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

        // Expose needed data
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

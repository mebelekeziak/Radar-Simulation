using System;
using System.IO;
using MoonSharp.Interpreter;
using MoonSharp.Interpreter.Loaders;
using RealRadarSim.Utils;

namespace RealRadarSim.Config
{
    /// <summary>
    /// Parses the <c>radar</c> table from <c>config.lua</c> into a strongly typed
    /// <see cref="RadarConfig"/> instance.  All validation and fall‑backs live
    /// in this helper so the main engine remains agnostic of Lua internals.
    /// </summary>
    public static class LuaConfigLoader
    {
        /// <summary>
        /// Reads the file, returns either a fully populated <see cref="RadarConfig"/>
        /// or <see cref="RadarConfig.Default"/> on any error.
        /// </summary>
        public static RadarConfig Load(string path)
        {
            if (!File.Exists(path))
                return RadarConfig.Default;

            try
            {
                Script script = new();
                var loader = new FileSystemScriptLoader { ModulePaths = new[] { "?.lua" } };
                script.Options.ScriptLoader = loader;
                script.DoFile(path);

                DynValue cfgDyn = script.Globals.Get("config");
                if (cfgDyn.Type == DataType.Table)
                {
                    DynValue radarDyn = cfgDyn.Table.Get("radar");
                    if (radarDyn.Type == DataType.Table)
                        return FromTable(radarDyn.Table);
                }
            }
            catch (Exception ex)
            {
                File.WriteAllText("lua_error.txt", $"Error loading {path}: {ex.Message}");
            }
            return RadarConfig.Default;
        }

        /// <summary>
        /// Converts a <see cref="Table"/> node (<c>config.radar</c>) into a
        /// typed record.  Used by unit tests to avoid hitting the file system.
        /// </summary>
        public static RadarConfig FromTable(Table tbl)
        {
            double N(string key, double def) => GetNumber(tbl, key, def);
            bool B(string key, bool def) => GetBool(tbl, key, def);

            return new RadarConfig
            {
                MaxRange = N("maxRange", 100_000),
                BeamWidthDeg = N("beamWidthDeg", 10.0),
                RotationSpeedDegSec = N("rotationSpeedDegSec", 36.0),
                FalseAlarmDensity = N("falseAlarmDensity", 1e-10),
                Snr0_dB = N("snr0_dB", 30.0),
                ReferenceRange = N("referenceRange", 10_000),
                RequiredSNR_dB = N("requiredSNR_dB", 0.0),
                RangeNoiseBase = N("rangeNoiseBase", 100.0),
                AngleNoiseBase = N("angleNoiseBase", 0.001),
                CfarWindowWidth = N("cfarWindowWidth", 5_000),
                CfarGuardWidth = N("cfarGuardWidth", 300),
                CfarThresholdMultiplier = N("cfarThresholdMultiplier", 8.0),
                ClusterDistanceMeters = N("clusterDistanceMeters", 600),
                RadarType = tbl.Get("radarType").CastToString() ?? "ground",
                AntennaElevationBars = (int)N("antennaElevationBars", 1),
                AntennaAzimuthScan = N("antennaAzimuthScan", 140),
                TiltOffsetDeg = N("tiltOffsetDeg", 0.0),
                LockRange = N("lockRange", 50_000.0),
                LockSNRThreshold_dB = N("lockSNRThreshold_dB", 5.0),
                PathLossExponent_dB = N("pathLossExponent_dB", 40.0),
                FrequencyHz = N("FrequencyHz", 3e9),
                TxPower_dBm = N("TxPower_dBm", 70.0),
                AntennaGain_dBi = N("AntennaGain_dBi", 101.0),
                ShowAzimuthBars = B("showAzimuthBars", false),
                ShowElevationBars = B("showElevationBars", false),
                UseDopplerProcessing = B("useDopplerProcessing", false),
                VelocityNoiseStd = N("velocityNoiseStd", 1.0),
                UseDopplerCFAR = B("useDopplerCFAR", false),
                DopplerCFARWindow = N("dopplerCFARWindow", 150),
                DopplerCFARGuard = N("dopplerCFARGuard", 20),
                DopplerCFARThresholdMultiplier = N("dopplerCFARThresholdMultiplier", 6.0),
                UseAesaMode = tbl.Get("operationMode").CastToString()?.ToLower() == "aesa"
            };
        }

        #region private helpers
        private static double GetNumber(Table tbl, string key, double def)
        {
            DynValue d = tbl.Get(key);
            return d.Type == DataType.Number ? d.Number : def;
        }

        private static bool GetBool(Table tbl, string key, bool def)
        {
            DynValue d = tbl.Get(key);
            return d.Type switch
            {
                DataType.Boolean => d.Boolean,
                DataType.Number => Math.Abs(d.Number) > double.Epsilon,
                _ => def
            };
        }
        #endregion
    }
}
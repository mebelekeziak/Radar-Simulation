**RealRadarSim (RadarEKF)**

- **Purpose:** Windows Forms radar simulation with detections, CFAR, JPDA multi-target tracking, and a maneuvering-target EKF. Designed for experimenting with radar/track-processing concepts and visualizing results in real time.
- **Tech:** C#/.NET 9, WinForms UI, MathNet.Numerics, MoonSharp (Lua config), KdTree for fast nearest-neighbor.
- **Outputs:** On-screen radar PPI with range rings, sweep/beam, detections, tracks, labels, and optional debug overlays.

**Highlights**

- **Radar models:** Ground (360° sweep) and aircraft-style scanning; optional AESA mode flag, antenna patterns (gaussian, sinc²), elevation “bars”, tilt, and configurable beamwidth/speed.
- **Signal model:** Advanced radar-equation SNR with RF losses and integration; optional simplified reference SNR model; off-boresight gain; per-detection SNR used for adaptive noise and gating.
- **Detections:** Range-domain CFAR with guard + window; optional Doppler-domain CFAR; clustering/merging of close detections in 3D.
- **Tracking:** JPDA association, SNR-aware gating, candidate initiation, pruning, and uncertainty-aware DBSCAN track merge; 9D constant-acceleration EKF with iterated updates and optional Doppler (range-rate) fusion.
- **UX:** Zoom, pan, debug overlay, per-track labels with heading and SNR, left-click lock (aircraft radar), unlock key.

**Quick Start**

- **Prereqs:**
  - `.NET 9 SDK` on Windows (WinForms target `net9.0-windows8.0`).
  - Visual Studio 2022 (17.12+) or `dotnet` CLI.
- **Run:**
  - VS: Open `RadarEKF.sln`, set `RadarEKF` as startup, Debug/Run.
  - CLI: From repo root: `dotnet run --project RadarEKF.csproj` (Windows required).
- **Controls:**
  - `Mouse wheel` zooms; `drag` pans; `+/-` zoom; `D` toggles debug; `U` unlocks; `Left‑click` a track to lock (aircraft radar type).

**Configuration**

- **File:** `config.lua` in repo root (optional). Safe defaults apply if absent or invalid.
- **Top-level tables:** `config.radar`, `config.targets`, `config.trackManager`.
- **Units:** Ranges in meters, angles in degrees (input) and radians (internal), speeds in m/s, powers in dBm/dBi, frequency in Hz.
- **Example:**

  ```lua
  config = {
    radar = {
      radarType = "aircraft",           -- "ground" | "aircraft"
      operationMode = "mechanical",     -- "mechanical" | "aesa"
      antennaPattern = "gaussian",      -- "gaussian" | "sinc2"
      maxRange = 100000,                 -- meters
      beamWidthDeg = 10.0,
      rotationSpeedDegSec = 36,
      tiltOffsetDeg = 0.0,
      antennaElevationBars = 4,
      antennaAzimuthScan = 140.0,
      falseAlarmDensity = 1e-10,
      -- RF/SNR
      FrequencyHz = 3e9,
      TxPower_dBm = 70.0,
      AntennaGain_dBi = 101.0,
      snr0_dB = 30.0,
      referenceRange = 10000.0,
      requiredSNR_dB = 0.0,
      pathLossExponent_dB = 40.0,
      -- Noise/CFAR/Clustering
      rangeNoiseBase = 100.0,
      angleNoiseBase = 0.001,
      cfarWindowWidth = 5000.0,
      cfarGuardWidth = 300.0,
      cfarThresholdMultiplier = 8.0,
      clusterDistanceMeters = 600.0,
      -- Doppler (optional)
      useDopplerProcessing = false,
      velocityNoiseStd = 1.0,
      useDopplerCFAR = false,
      dopplerCFARWindow = 150.0,
      dopplerCFARGuard = 20.0,
      dopplerCFARThresholdMultiplier = 6.0,
      -- Lock behavior
      lockRange = 50000.0,
      lockSNRThreshold_dB = 5.0,
      -- UI
      showAzimuthBars = false,
      showElevationBars = false,
    },

    targets = {
      { x = 20000, y = 15000, z = 2000, speed = 180, headingDeg = 45,  climbRate = 5,  turnRateDeg = 2, processStd = 0.5, aircraftName = "Boeing 737",  rcs = 40.0 },
      { x = -25000, y = -20000, z = 1000, speed = 220, headingDeg = -60, climbRate = 0,  turnRateDeg = 3, processStd = 0.5, aircraftName = "Airbus A320", rcs = 30.0 },
      { x = 10000, y = -10000, z = 3000, speed = 120, headingDeg = 90,  climbRate = -5, turnRateDeg = 1, processStd = 0.3, aircraftName = "Boeing 777",  rcs = 60.0 },
      { x = 60000, y = -45000, z = 3000, speed = 270, headingDeg = 90,  climbRate = 15, turnRateDeg = 1, processStd = 0.3, aircraftName = "F-16",       rcs = 25.0 },
    },

    trackManager = {
      initGateThreshold = 14.07,      -- χ² gate for seeding
      initRequiredHits = 3,
      initScanWindow = 3,
      initPosStd = 200.0,             -- meters
      initVelStd = 100.0,             -- m/s
      gatingProbability = 0.99,       -- χ² base for JPDA gates
      accelNoise = 2.0,
      probDetection = 0.9,
      probSurvival = 0.995,
      pruneThreshold = 0.01,
      maxTrackMergeDist = 800.0,
      maxTrackAge = 40.0,             -- seconds since last update
      candidateMergeDistance = 1500.0,
    }
  }
  ```

- **Notes:**
  - Only a subset is required unspecified fields fall back to defaults.
  - `radarType = "aircraft"` enables bar-scanning and locking `Left-click` on a track to command a lock, `U` to unlock.
  - Set `useDopplerProcessing = true` to add per‑detection radial velocity and enable Doppler-aware tracking.

**UI & Interaction**

- **Display:** Range rings and axes sweep/beam true target dots confirmed tracks as orange dots with heading tick labels show `TrackId`, flight name, existence probability, heading, and instantaneous SNR.
- **Mouse:** Wheel zoom left‑drag pan hover to see target tooltips (range/altitude/velocity).
- **Keyboard:** `D` toggles debug (shows raw measurements), `+/-` zoom, `U` unlocks.
- **Locking:** In aircraft mode, click a track to command the sensor to slew and hold on it while within `lockRange` and SNR threshold.

**Project Structure**

- `Engine/SimulationEngine.cs`: Orchestrates targets → radar → detections → tracking; time step; KD‑tree for flight‑name assignment.
- `Models/AdvancedRadar.*`: Beam update, SNR and RF math, CFAR (range + optional Doppler), detection clustering; antenna pattern and sweep models.
- `Tracking/TrackManager.cs`: JPDA gating/association, candidate initiation/confirmation, pruning, duplicate suppression, DBSCAN‑based track merging.
- `Tracking/ExtendedKalmanFilter.cs`: 9D constant‑acceleration EKF with iterated updates; optional 4D update with Doppler (range‑rate).
- `Forms/RadarForm.*`: WinForms visualization and interaction (zoom/pan/lock/debug overlay/labels).
- `Config/*`: Strongly‑typed config record and Lua loader; optional `config.lua` at repo root.
- `Utils/MathUtil.cs`: Angle helpers and small math utilities.

**Radar & Tracking Details**

- **SNR model:** Choice between full radar equation (λ, Pt, G, RCS, losses, bandwidth, noise figure, pulse integration) and reference‑SNR model; off‑boresight attenuation via gaussian or sinc² pattern.
- **CFAR:** Range CFAR with guard + window and percentile‑based threshold; optional Doppler CFAR with independent parameters.
- **Clustering:** 3D Cartesian distance merge with range‑scaled angular tolerance; chooses the strongest amplitude in a cluster.
- **JPDA:** SNR‑aware gating, numerically stable likelihoods (Cholesky), β/β0 with clutter density, existence‑probability dynamics.
- **EKF:** Iterated update; adaptive process noise from innovation magnitude; Doppler fusion path provides 4D measurement model and Jacobian.

**Build Notes**

- **SDK:** Ensure `dotnet --list-sdks` shows a 9.0 SDK; WinForms target requires Windows.
- **Packages:** Restored via NuGet (see `RadarEKF.csproj`): `MathNet.Numerics`, `MoonSharp`, `KdTree`.
- **CLI commands:**
  - Restore: `dotnet restore`
  - Build: `dotnet build -c Debug`
  - Run: `dotnet run --project RadarEKF.csproj`
- **Artifacts:** Binaries under `bin/Debug/net9.0-windows8.0/`.

**Logging**

- **Runtime:** In‑memory rolling log exposed by `RealRadarSim.Logging.DebugLogger.DebugMessages`.
- **File output:** Disabled by default. Enable by setting `DebugLogger.EnableFileLogging = true;` at startup to write `debug.txt` with rotation.

**Troubleshooting**

- **Black window / no tracks:** Check `config.lua` ranges vs `maxRange`, and `requiredSNR_dB` vs scenario SNR; toggle `D` to verify raw measurements.
- **No Doppler:** Ensure `useDopplerProcessing = true` and that targets have non‑zero speed/climb; Doppler CFAR is optional and separate.
- **Build errors on Linux/macOS:** Target is `net9.0-windows8.0` due to WinForms; run on Windows, or port UI to cross‑platform UI if needed.

**Credits**

- Math primitives by MathNet.Numerics. Lua parsing via MoonSharp. KD‑tree via KdTree. All trademarks belong to their owners.


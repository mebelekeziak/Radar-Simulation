config = {
  radar = {
      maxRange = 120000,              -- Maximum detection range in meters
      beamWidthDeg = 12.0,             -- Beam width in degrees
      rotationSpeedDegSec = 36,       -- Beam rotation speed in degrees per second
      falseAlarmDensity = 1e-10,      -- False alarm density
      snr0 = 20000.0,                 -- Reference SNR value
      referenceRange = 20000.0,       -- Reference range in meters
      requiredSNR = 0.003,              -- Minimum SNR required to detect a target
      rangeNoiseBase = 80.0,          -- Base noise level for range measurements
      angleNoiseBase = 0.0005,        -- Base noise level for angle measurements
      cfarWindowWidth = 4000.0,        -- CFAR algorithm window width
      cfarGuardWidth = 250.0,          -- CFAR guard width
      cfarThresholdMultiplier = 7.0,   -- CFAR threshold multiplier
      clusterDistanceMeters = 500.0,   -- Distance threshold for clustering detections

      -- Radar type (can be "ground" or "aircraft")
      radarType = "aircraft",

      -- Optional display settings (for visualization purposes)
      showAzimuthBars = true,
      showElevationBars = true,

      -- Horizontal and bar configuration (used for "aircraft" radar type)
      antennaAzimuthScan = 140,    -- Azimuth scan width in degrees
      antennaElevationBars = 4,        -- Number of elevation bars (this becomes the antenna height)
      barSpacingDeg = 1.0,             -- Spacing between bars in degrees

      -- Tilt offset is applied to the elevation angle (in degrees)
      tiltOffsetDeg = 2.0
  },

  targets = {
      {
          x = 20000, y = 15000, z = 2000,
          speed = 180, headingDeg = 45,
          climbRate = 5, turnRateDeg = 2,
          processStd = 0.5,
          aircraftName = "Boeing 737",
          rcs = 40.0
      },
      {
          x = -25000, y = -20000, z = 1000,
          speed = 220, headingDeg = -60,
          climbRate = 0, turnRateDeg = 3,
          processStd = 0.5,
          aircraftName = "Airbus A320",
          rcs = 30.0
      },
      {
          x = 10000, y = -10000, z = 3000,
          speed = 120, headingDeg = 90,
          climbRate = -5, turnRateDeg = 1,
          processStd = 0.3,
          aircraftName = "Boeing 777",
          rcs = 60.0
      },
      {
          x = 60000, y = -45000, z = 3000,
          speed = 270, headingDeg = 90,
          climbRate = 15, turnRateDeg = 1,
          processStd = 0.3,
          aircraftName = "F-16",
          rcs = 25.0
      }
  },

  trackManager = {
      initGateThreshold = 14.07,
      initRequiredHits = 2,
      initScanWindow = 3,
      initPosStd = 200.0,
      initVelStd = 100.0,
      gatingThreshold = 14.07,
      accelNoise = 2.0,
      probDetection = 0.9,
      probSurvival = 0.995,
      pruneThreshold = 0.01,
      maxTrackMergeDist = 800.0,
      maxTrackAge = 20.0,
      candidateMergeDistance = 1500.0
  }
}

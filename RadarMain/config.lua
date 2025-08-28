config = {
  radar = {
    maxRange = 120000,              -- Maximum detection range in meters
    beamWidthDeg = 7.5,             -- Beam width in degrees
    rotationSpeedDegSec = 36,       -- Beam rotation speed in degrees per second
    falseAlarmDensity = 1e-09,      -- False alarm density

    useDopplerProcessing = true,
    velocityNoiseStd = 1.5,
    useDopplerCFAR = true,
    dopplerCFARWindow = 600.0,
    dopplerCFARGuard = 150.5,
    dopplerCFARThresholdMultiplier = 5,

    FrequencyHz = 3e9,              -- Operating frequency (Hz)
    TxPower_dBm = 88,               -- Transmit power (dBm)
    AntennaGain_dBi = 36,           -- Antenna gain (dBi)

    -- SNR parameters in dB:
    snr0_dB = 30.0,                 -- Reference SNR (for 1 m² RCS target at reference range)
    referenceRange = 20000.0,       -- Reference range (m)
    requiredSNR_dB = 6,             -- Minimum SNR required for detection

    lockRange = 70000,              -- Lock range (m)
    lockSNRThreshold_dB = 3.0,      -- Lock threshold (dB)

    rangeNoiseBase = 30.0,          -- Base noise (m)
    angleNoiseBase = 0.0005,        -- Base angular noise (rad)

    cfarWindowWidth = 600.0,        -- CFAR window (m)
    cfarGuardWidth = 150.0,         -- CFAR guard (m)
    cfarThresholdMultiplier = 5.0,  -- CFAR multiplier
    clusterDistanceMeters = 3000.0, -- Clustering distance (m)

    -- Radar type (can be "ground" or "aircraft")
    radarType = "ground",

    -- Supported: "gaussian", "sinc2"
    antennaPattern = "gaussian",

    -- Display options
    showAzimuthBars = true,
    showElevationBars = true,

    -- Aircraft radar settings
    antennaAzimuthScan = 140,       -- Azimuth scan (deg)
    antennaElevationBars = 4,       -- Number of elevation bars
    barSpacingDeg = 1.0,            -- Elevation bar spacing (deg)

    tiltOffsetDeg = 10,             -- Elevation tilt offset (deg)

    -- Path loss exponent (dB)
    pathLossExponent_dB = 40.0
  },

  targets = {
    {
      x = 20000, y = 15000, z = 2000,
      speed = 180, headingDeg = 45,
      climbRate = 5, turnRateDeg = 0.5,
      processStd = 0.5,
      aircraftName = "Boeing 747",
      rcs = 15.0
    },
    {
      x = -25000, y = -20000, z = 1000,
      speed = 220, headingDeg = -60,
      climbRate = 0, turnRateDeg = 3,
      processStd = 0.5,
      aircraftName = "Airbus A320",
      rcs = 15.0
    },
    {
      x = 50000, y = 40000, z = 10000,
      speed = 220, headingDeg = -60,
      climbRate = 0, turnRateDeg = 0,
      processStd = 0.0,
      aircraftName = "MiG-29",
      rcs = 7.0
    },
    {
      x = 10000, y = -10000, z = 3000,
      speed = 120, headingDeg = 90,
      climbRate = -5, turnRateDeg = 1,
      processStd = 0.3,
      aircraftName = "Boeing 777",
      rcs = 15.0
    },
    {
      x = 5000, y = -25000, z = 1000,
      speed = 270, headingDeg = 90,
      climbRate = 15, turnRateDeg = 1,
      processStd = 0.3,
      aircraftName = "F-35",
      rcs = 0.0001
    }
  },

  trackManager = {
    initGateThreshold = 22.07,
    initRequiredHits = 2,
    initScanWindow = 6,
    initPosStd = 60.0,
    initVelStd = 5.0,
    gatingProbability = 0.99,       -- prefer probability instead of threshold
    accelNoise = 2.0,
    probDetection = 0.99,
    probSurvival = 0.995,
    pruneThreshold = 0.001,
    maxTrackMergeDist = 6000.0,
    maxTrackAge = 20,
    candidateMergeDistance = 5000.0
  },
}

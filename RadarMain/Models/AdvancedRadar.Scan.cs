using System;
using RealRadarSim.Tracking;
using RealRadarSim.Utils;

namespace RealRadarSim.Models
{
    public partial class AdvancedRadar
    {
        // ———————————————————————————————————  public surface  ——————————————————————————————————
        public void LockTarget(JPDA_Track track)
        {
            if (RadarType == "aircraft")
                lockedTrack = track;
        }

        public void UnlockTarget() => lockedTrack = null;

        public void UpdateBeam(double dt)
        {
            double speedMul = UseAesaMode ? BeamSpeedMultiplier : 1.0;

            if (RadarType == "aircraft")
            {
                // — Prioritised track lock —
                if (lockedTrack is not null)
                {
                    TrackLockedTrack(dt);
                    return;
                }

                if (UseAesaMode)
                {
                    if (AesaBeams is { Count: > 0 })
                    {
                        foreach (var b in AesaBeams)
                            b.Update(dt, rotationSpeedRadSec * speedMul, aesaElevationOscFreq);
                    }
                }
                else
                {
                    double dAz = rotationSpeedRadSec * speedMul * dt * (scanLeftToRight ? 1 : -1);
                    CurrentAzimuth += dAz;

                    if (scanLeftToRight && CurrentAzimuth >= maxAzimuth)
                    {
                        CurrentAzimuth = maxAzimuth;
                        scanLeftToRight = false;
                        AdvanceElevationBar();
                    }
                    else if (!scanLeftToRight && CurrentAzimuth <= minAzimuth)
                    {
                        CurrentAzimuth = minAzimuth;
                        scanLeftToRight = true;
                        AdvanceElevationBar();
                    }
                }
            }
            else // ground‑based 360° sweep
            {
                CurrentBeamAngle += rotationSpeedRadSec * speedMul * dt;
                if (CurrentBeamAngle > 2 * Math.PI)
                    CurrentBeamAngle -= 2 * Math.PI;
            }
        }

        // ———————————————————————————————————  private helpers  ——————————————————————————————————
        private void AdvanceElevationBar()
        {
            currentElevationBar = (currentElevationBar + 1) % AntennaHeight;
            double barSpacingRad = MathUtil.DegToRad(2.0);
            CurrentElevation = minElevation + currentElevationBar * barSpacingRad;
        }

        private void TrackLockedTrack(double dt)
        {
            double x = lockedTrack.Filter.State[0];
            double y = lockedTrack.Filter.State[1];
            double z = lockedTrack.Filter.State[2];
            double r = Math.Sqrt(x * x + y * y + z * z);
            if (r > lockRange)
            {
                UnlockTarget();
                return;
            }

            double desiredAz = Math.Atan2(y, x);
            double desiredEl = Math.Atan2(z, Math.Sqrt(x * x + y * y));
            double maxStep = rotationSpeedRadSec * dt;

            double dAz = MathUtil.NormalizeAngle(desiredAz - CurrentAzimuth);
            CurrentAzimuth += Math.Clamp(dAz, -maxStep, maxStep);
            CurrentAzimuth = MathUtil.NormalizeAngle(CurrentAzimuth);

            double dEl = MathUtil.NormalizeAngle(desiredEl - CurrentElevation);
            CurrentElevation += Math.Clamp(dEl, -maxStep, maxStep);
            CurrentElevation = MathUtil.NormalizeAngle(CurrentElevation);
        }
    }
}
//using System;
//using System.Numerics;
//using RealRadarSim.Models;

//namespace RadarEKF.Missile
//{

//    internal class Navigation
//    {
//        private AdvancedRadar radar;
//        private float N = 32.0f;             
//        private float maxTurnRate = 4.5f;       
//        private Vector3? previousLOS = null;   

//        public Navigation(AdvancedRadar radar)
//        {
//            this.radar = radar;
//        }

//        public bool CanFireMissile()
//        {
//            return radar.IsLocked;
//        }


//        public Vector3 GetGuidanceCommand(Vector3 missilePosition, Vector3 missileVelocity, Vector3 currentThrustDirection, float dt)
//        {
//            if (!radar.IsLocked)
//            {
//                previousLOS = null;
//                return currentThrustDirection;
//            }

//            Vector3? targetPosNullable = radar.GetLockedTargetPosition();
//            if (targetPosNullable == null)
//            {
//                previousLOS = null;
//                return currentThrustDirection;
//            }
//            Vector3 targetPos = targetPosNullable.Value;

//            // Compute the line-of-sight (LOS) vector.
//            Vector3 losVector = targetPos - missilePosition;
//            float distance = losVector.Length();
//            if (distance < 1e-6f)
//            {
//                previousLOS = null;
//                return currentThrustDirection;
//            }
//            Vector3 los = Vector3.Normalize(losVector);

//            // -------- Terminal Phase Guidance --------
//            float terminalDistance = 5000f; // e.g., within 5 km, start terminal guidance.
//            if (distance < terminalDistance)
//            {
//                // Compute the proportional navigation (PN) command.
//                float dot = Math.Clamp(Vector3.Dot(currentThrustDirection, los), -1f, 1f);
//                float angularError = (float)Math.Acos(dot);
//                Vector3 rotationAxis = Vector3.Cross(currentThrustDirection, los);
//                if (rotationAxis.LengthSquared() < 1e-9f)
//                    rotationAxis = Vector3.UnitY;
//                else
//                    rotationAxis = Vector3.Normalize(rotationAxis);
//                float turnAnglePN = Math.Min(maxTurnRate * dt, N * angularError);
//                Vector3 pnCommand = RotateVector(currentThrustDirection, rotationAxis, turnAnglePN);

//                // Compute a braking command (opposite to missile velocity).
//                Vector3 brakingCommand = (missileVelocity.Length() > 1e-6f)
//                    ? -Vector3.Normalize(missileVelocity)
//                    : pnCommand;

//                // Blend the PN and braking commands as the missile nears the target.
//                float blendRatio = (float)Math.Pow(1.0f - (distance / terminalDistance), 3);
//                if (distance < 2000f)
//                    blendRatio = Math.Max(blendRatio, 0.8f);
//                Vector3 blendedCommand = Vector3.Lerp(pnCommand, brakingCommand, blendRatio);
//                return Vector3.Normalize(blendedCommand);
//            }

//            // -------- Standard Proportional Navigation (PN) --------
//            float dotStandard = Math.Clamp(Vector3.Dot(currentThrustDirection, los), -1f, 1f);
//            float angularErrorStandard = (float)Math.Acos(dotStandard);
//            Vector3 rotationAxisStandard = Vector3.Cross(currentThrustDirection, los);
//            if (rotationAxisStandard.LengthSquared() < 1e-9f)
//                return currentThrustDirection;
//            rotationAxisStandard = Vector3.Normalize(rotationAxisStandard);
//            float turnAngleStandard = Math.Min(maxTurnRate * dt, N * angularErrorStandard);
//            Vector3 newThrust = RotateVector(currentThrustDirection, rotationAxisStandard, turnAngleStandard);

//            return Vector3.Normalize(newThrust);
//        }


//        private Vector3 RotateVector(Vector3 v, Vector3 axis, float angle)
//        {
//            float cosTheta = (float)Math.Cos(angle);
//            float sinTheta = (float)Math.Sin(angle);
//            return v * cosTheta
//                   + Vector3.Cross(axis, v) * sinTheta
//                   + axis * (Vector3.Dot(axis, v) * (1 - cosTheta));
//        }
//    }
//}

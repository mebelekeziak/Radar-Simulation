//using System;
//using System.Collections.Generic;
//using System.Numerics; // For Vector3

//namespace RadarEKF.Aerodynamics
//{
//    public struct FlightState
//    {
//        public double Time;       // Time in seconds
//        public Vector3 Position;  // Position (X, Y, Z) in meters
//        public Vector3 Velocity;  // Velocity (m/s)
//    }

//    internal class FlightModel
//    {
//        // --- Missile Parameters ---
//        public float Mass { get; set; } = 400.0f;
//        public float Thrust { get; set; } = 30000.0f;
//        public float BoostDuration { get; set; } = 12.0f;

//        // --- Aerodynamics ---
//        public float DragCoefficient { get; set; } = 0.3f;
//        public float CrossSectionalArea { get; set; } = 0.0314f;
//        public float LiftCoefficient { get; set; } = 0.2f;      // Was 0.5f
//        public float WingArea { get; set; } = 0.2f;             // Was 0.5f

//        // --- Environment ---
//        public float AirDensity { get; set; } = 1.225f;
//        public float Gravity { get; set; } = 9.81f;

//        // --- Initial Conditions ---
//        public Vector3 InitialPosition { get; set; } = new Vector3(0, 0, 0);
//        public Vector3 InitialVelocity { get; set; }
//        public float LaunchAngle { get; set; } = 45.0f;
//        public Vector3 ThrustDirection { get; set; }

//        public float InitialSpeed { get; set; } = 100.0f;

//        public FlightModel()
//        {
//            float angleRad = LaunchAngle * (float)Math.PI / 180.0f;
//            InitialVelocity = new Vector3(
//                (float)Math.Cos(angleRad),
//                0f,
//                (float)Math.Sin(angleRad)
//            ) * InitialSpeed;
//            ThrustDirection = Vector3.Normalize(InitialVelocity);
//        }

//        public List<FlightState> SimulateFlight(float dt, float totalTime, RadarEKF.Missile.Navigation navigation = null)
//        {
//            List<FlightState> states = new List<FlightState>();

//            Vector3 position = InitialPosition;
//            Vector3 velocity = InitialVelocity;
//            float time = 0.0f;

//            while (time <= totalTime)
//            {

//                if (position.Z < 0)
//                    break;

//                if (navigation != null)
//                {
//                    ThrustDirection = navigation.GetGuidanceCommand(position, velocity, ThrustDirection, dt);
//                }

//                float currentThrust = (time < BoostDuration) ? Thrust : 0.0f;
//                Vector3 aThrust = (currentThrust / Mass) * ThrustDirection;

//                float speed = velocity.Length();
//                Vector3 aDrag = Vector3.Zero;
//                if (speed > 0)
//                {
//                    float dragForce = 0.5f * AirDensity * speed * speed * DragCoefficient * CrossSectionalArea;
//                    aDrag = -(dragForce / Mass) * Vector3.Normalize(velocity);
//                }

               
//                Vector3 aGravity = new Vector3(0, 0, -Gravity);

//                // lift
//                Vector3 aLift = Vector3.Zero;
//                if (speed > 1e-3)
//                {
//                    float q = 0.5f * AirDensity * speed * speed;
//                    float liftForce = q * WingArea * LiftCoefficient;

//                    Vector3 up = new Vector3(0f, 0f, 1f);
//                    Vector3 side = Vector3.Cross(velocity, up);
//                    if (side.LengthSquared() > 1e-6f)
//                    {
//                        side = Vector3.Normalize(side);
//                        Vector3 liftDir = Vector3.Cross(side, velocity);
//                        liftDir = Vector3.Normalize(liftDir);

//                        aLift = (liftForce / Mass) * liftDir;
//                    }
//                }

//                // acceleration
//                Vector3 acceleration = aThrust + aDrag + aGravity + aLift;

//                // --- Prosta metoda całkowania Eulera ---
//                velocity += acceleration * dt;
//                position += velocity * dt;

//                // save state
//                states.Add(new FlightState
//                {
//                    Time = time,
//                    Position = position,
//                    Velocity = velocity
//                });

//                time += dt;
//            }

//            return states;
//        }
//    }
//}

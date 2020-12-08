using KrpcAutoPilot.Utils;

namespace KrpcAutoPilot.Data
{
    public class Body
    {
        /// <summary>
        /// The depth of the atmosphere, in meters.
        /// </summary>
        public double AtmosphereDepth { get; set; }
        /// <summary>
        /// The standard gravitational parameter of the body in m^3s^{-2}.
        /// </summary>
        public double GravitationalParameter { get; set; }
        public bool HasAtmosphere { get; set; }
        public double Radius { get; set; }
    }

    public class Environment
    {
        /// <summary>
        /// The static atmospheric pressure acting on the vessel, in Pascals.
        /// </summary>
        public double StaticPressure { get; set; }
        /// <summary>
        /// The static (ambient) temperature of the atmosphere around the vessel, in Kelvin.
        /// </summary>
        public double Temperature { get; set; }
    }

    public class Orbit
    {
        public double Apoapsis { get; set; }
        public double Periapsis { get; set; }
        /// <summary>
        /// The time until the object reaches apoapsis, in seconds.
        /// </summary>
        public double TimeToApoapsis { get; set; }
    }

    public class Vessel
    {
        public double Altitude { get; set; }
        /// <summary>
        /// The angular velocity as a vector. The magnitude of the vector is the rotational
        /// speed of the vessel, in radians per second. The direction of the vector indicates
        /// the axis of rotation, using the right-hand rule.
        /// </summary>
        public Vector3d AngularVelocity { get; set; }
        /// <summary>
        /// The maximum force that the currently active RCS thrusters can generate. Returns
        /// the forces in N along each of the coordinate axes of the vessels reference frame
        /// (SpaceCenter.ReferenceFrame). These axes are equivalent to the right, forward
        /// and bottom directions of the vessel.
        /// </summary>
        public TupleV3d AvailableRCSForce { get; set; }
        /// <summary>
        /// Gets the total available thrust that can be produced by the vessel's active engines, in Newtons.
        /// </summary>
        public double AvailableThrust { get; set; }
        /// <summary>
        /// The maximum torque that the vessel generates. Includes contributions from reaction
        /// wheels, RCS, gimballed engines and aerodynamic control surfaces. Returns the
        /// torques in N.m around each of the coordinate axes of the vessels reference frame
        /// (SpaceCenter.ReferenceFrame). These axes are equivalent to the pitch, roll and
        /// yaw axes of the vessel.
        /// </summary>
        public TupleV3d AvailableTorque { get; set; }
        public Vector3d Direction { get; set; }
        /// <summary>
        /// The total mass of the vessel, excluding resources, in kg.
        /// </summary>
        public double DryMass { get; set; }
        public Vector3d Forward { get; set; }
        public double Gravity { get; set; }
        /// <summary>
        /// The total mass of the vessel, including resources, in kg.
        /// </summary>
        public double Mass { get; set; }
        /// <summary>
        /// Throttle 为 1 时消耗燃料的速度，kg/s
        /// </summary>
        public double MaxFuelRate { get; set; }
        /// <summary>
        /// The total maximum thrust that can be produced by the vessel's active engines
        /// when the vessel is in a vacuum, in Newtons. This is computed by summing
        /// SpaceCenter.Engine.MaxVacuumThrust for every active engine.
        /// </summary>
        public double MaxVacuumThrust { get; set; }
        /// <summary>
        /// The moment of inertia of the vessel around its center of mass in kg.m^2. The
        /// inertia values in the returned 3-tuple are around the pitch, roll and yaw directions
        /// respectively. This corresponds to the vessels reference frame (SpaceCenter.ReferenceFrame).
        /// </summary>
        public Vector3d MomentOfInertia { get; set; }
        public Vector3d Position { get; set; }
        public Vector3d Right { get; set; }
        public Vector3d SurfEast { get; set; }
        public Vector3d SurfNorth { get; set; }
        public Vector3d SurfUp { get; set; }
        /// <summary>
        /// The total thrust currently being produced by the vessel's engines, in Newtons.
        /// This is computed by summing SpaceCenter.Engine.Thrust for every engine in the
        /// vessel.
        /// </summary>
        public double Thrust { get; set; }
        public Vector3d Up { get; set; }
        /// <summary>
        /// The combined vacuum specific impulse of all active engines, in seconds. This
        /// is computed using the formula described here.
        /// </summary>
        public double VacuumSpecificImpulse { get; set; }
        public Vector3d Velocity { get; set; }
        public Vector3d VelocityHorizon { get; set; }
        public double VelocityHorizonMag { get; set; }
        public double VelocityMag { get; set; }
        public double VelocityUp { get; set; }
    }
}

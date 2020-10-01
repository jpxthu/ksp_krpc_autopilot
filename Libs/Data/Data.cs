using KRPCLibs.Math;

namespace KRPCLibs.Data
{
    public class Body
    {
        /// <summary>
        /// The standard gravitational parameter of the body in m^3s^{-2}.
        /// </summary>
        public double GravitationalParameter { get; set; }
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
        /// Gets the total available thrust that can be produced by the vessel's active engines, in Newtons.
        /// </summary>
        public double AvailableThrust { get; set; }
        /// <summary>
        /// The total mass of the vessel, including resources, in kg.
        /// </summary>
        public double Mass { get; set; }
        public Vector3d Velocity { get; set; }
        public double VelocityMag { get; set; }
        public Vector3d Position { get; set; }
    }
}

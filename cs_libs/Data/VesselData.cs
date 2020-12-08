using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot.Data
{
    public class VesselStreams
    {
        public VesselStreams(Connection conn, Service sc, KRPC.Client.Services.SpaceCenter.Vessel active_vessel)
        {
            var flight = active_vessel.Flight();
            var orbit = active_vessel.Orbit;
            var body = orbit.Body;

            // Body
            AtmosphereDepth = conn.AddStream(() => body.AtmosphereDepth);
            GravitationalParameter = conn.AddStream(() => body.GravitationalParameter);
            HasAtmosphere = conn.AddStream(() => body.HasAtmosphere);
            Radius = conn.AddStream(() => body.EquatorialRadius);
            // Environment
            StaticAirTemperature = conn.AddStream(() => flight.StaticAirTemperature);
            StaticPressure = conn.AddStream(() => flight.StaticPressure);
            // Orbit
            Apoapsis = conn.AddStream(() => orbit.Apoapsis);
            Periapsis = conn.AddStream(() => orbit.Periapsis);
            TimeToApoapsis = conn.AddStream(() => orbit.TimeToApoapsis);
            // Vessel
            Altitude = conn.AddStream(() => flight.MeanAltitude);
            AngularVelocity = conn.AddStream(() => active_vessel.AngularVelocity(body.ReferenceFrame));
            AvailableRCSForce = conn.AddStream(() => active_vessel.AvailableRCSForce);
            AvailableThrust = conn.AddStream(() => active_vessel.AvailableThrust);
            AvailableTorque = conn.AddStream(() => active_vessel.AvailableTorque);
            Direction = conn.AddStream(() => active_vessel.Direction(body.ReferenceFrame));
            DryMass = conn.AddStream(() => active_vessel.DryMass);
            Forward = conn.AddStream(() => sc.TransformDirection(
                new Tuple<double, double, double>(0, 1, 0),
                active_vessel.ReferenceFrame, body.ReferenceFrame));
            Mass = conn.AddStream(() => active_vessel.Mass);
            MaxVacuumThrust = conn.AddStream(() => active_vessel.MaxVacuumThrust);
            MomentOfInertia = conn.AddStream(() => active_vessel.MomentOfInertia);
            Position = conn.AddStream(() => active_vessel.Position(body.ReferenceFrame));
            Right = conn.AddStream(() => sc.TransformDirection(
                new Tuple<double, double, double>(1, 0, 0),
                active_vessel.ReferenceFrame, body.ReferenceFrame));
            Thrust = conn.AddStream(() => active_vessel.Thrust);
            Up = conn.AddStream(() => sc.TransformDirection(
                new Tuple<double, double, double>(0, 0, -1),
                active_vessel.ReferenceFrame, body.ReferenceFrame));
            VacuumSpecificImpulse = conn.AddStream(() => active_vessel.VacuumSpecificImpulse);
            Velocity = conn.AddStream(() => active_vessel.Velocity(body.ReferenceFrame));
        }

        // Body
        public Stream<float> AtmosphereDepth { get; }
        public Stream<float> GravitationalParameter { get; }
        public Stream<bool> HasAtmosphere { get; }
        public Stream<float> Radius { get; }
        // Environment
        public Stream<float> StaticAirTemperature { get; }
        public Stream<float> StaticPressure { get; }
        // Orbit
        public Stream<double> Apoapsis { get; }
        public Stream<double> Periapsis { get; }
        public Stream<double> TimeToApoapsis { get; }
        // Vessel
        public Stream<double> Altitude { get; }
        public Stream<Tuple<double, double, double>> AngularVelocity { get; }
        public Stream<Tuple<Tuple<double, double, double>, Tuple<double, double, double>>> AvailableRCSForce { get; }
        public Stream<float> AvailableThrust { get; }
        public Stream<Tuple<Tuple<double, double, double>, Tuple<double, double, double>>> AvailableTorque { get; }
        public Stream<Tuple<double, double, double>> Direction { get; }
        public Stream<float> DryMass { get; }
        public Stream<Tuple<double, double, double>> Forward { get; }
        public Stream<float> Mass { get; }
        public Stream<float> MaxVacuumThrust { get; }
        public Stream<Tuple<double, double, double>> MomentOfInertia { get; }
        public Stream<Tuple<double, double, double>> Position { get; }
        public Stream<Tuple<double, double, double>> Right { get; }
        public Stream<float> Thrust { get; }
        public Stream<Tuple<double, double, double>> Up { get; }
        public Stream<float> VacuumSpecificImpulse { get; }
        public Stream<Tuple<double, double, double>> Velocity { get; }
    }

    public class VesselData
    {
        public void Update()
        {
            // Body
            Body.AtmosphereDepth = Streams.AtmosphereDepth.Get();
            Body.GravitationalParameter = Streams.GravitationalParameter.Get();
            Body.HasAtmosphere = Streams.HasAtmosphere.Get();
            Body.Radius = Streams.Radius.Get();

            // Environment
            Environment.Temperature = Streams.StaticAirTemperature.Get();
            Environment.StaticPressure = Streams.StaticPressure.Get();

            // Orbit
            Orbit.Apoapsis = Streams.Apoapsis.Get();
            Orbit.Periapsis = Streams.Periapsis.Get();
            Orbit.TimeToApoapsis = Streams.TimeToApoapsis.Get();

            // Vessel
            Vessel.Altitude = Streams.Altitude.Get();
            Vessel.AngularVelocity = new Vector3d(Streams.AngularVelocity.Get());
            Vessel.AvailableRCSForce = new TupleV3d(Streams.AvailableRCSForce.Get());
            Vessel.AvailableThrust = Streams.AvailableThrust.Get();
            Vessel.AvailableTorque = new TupleV3d(Streams.AvailableTorque.Get());
            Vessel.Direction = new Vector3d(Streams.Direction.Get());
            Vessel.DryMass = Streams.DryMass.Get();
            Vessel.Forward = new Vector3d(Streams.Forward.Get());
            Vessel.Mass = Streams.Mass.Get();
            Vessel.MaxVacuumThrust = Streams.MaxVacuumThrust.Get();
            Vessel.MomentOfInertia = new Vector3d(Streams.MomentOfInertia.Get());
            Vessel.Position = new Vector3d(Streams.Position.Get());
            Vessel.Right = new Vector3d(Streams.Right.Get());
            Vessel.Thrust = Streams.Thrust.Get();
            Vessel.Up = new Vector3d(Streams.Up.Get());
            Vessel.VacuumSpecificImpulse = Streams.VacuumSpecificImpulse.Get();
            Vessel.Velocity = new Vector3d(Streams.Velocity.Get());

            Vessel.Gravity = Body.GravitationalParameter / Vessel.Position.LengthSquared();
            Vessel.MaxFuelRate = Vessel.MaxVacuumThrust / Vessel.VacuumSpecificImpulse / Constants.Common.GRAVITY_ON_KERBIN;
            Vessel.SurfUp = Vessel.Position.Norm();
            Vessel.SurfEast = Vector3d.Cross(Vessel.SurfUp, new Vector3d(0d, 1d, 0d)).Norm();
            Vessel.SurfNorth = Vector3d.Cross(Vessel.SurfEast, Vessel.SurfUp);
            Vessel.VelocityHorizon = Vessel.Velocity - Vessel.Velocity * Vessel.SurfUp * Vessel.SurfUp;
            Vessel.VelocityHorizonMag = Vessel.VelocityHorizon.Length();
            Vessel.VelocityMag = Vessel.Velocity.Length();
            Vessel.VelocityUp = Vessel.Velocity * Vessel.SurfUp;

            Available = true;
        }

        public VesselData(Connection conn, Service sc, KRPC.Client.Services.SpaceCenter.Vessel active_vessel)
        {
            Streams = new VesselStreams(conn, sc, active_vessel);
            Body = new Body();
            Environment = new Environment();
            Orbit = new Orbit();
            Vessel = new Vessel();
            Available = false;
        }

        private VesselStreams Streams { get; }
        public Body Body { get; private set; }
        public Environment Environment { get; private set; }
        public Orbit Orbit { get; private set; }
        public Vessel Vessel { get; private set; }
        public bool Available { get; private set; }
    }
}

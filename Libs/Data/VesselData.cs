using KRPC.Client;
using KRPCLibs.Math;
using System;

namespace KRPCLibs.Data
{
    public class VesselStreams
    {
        public VesselStreams(Connection conn, KRPC.Client.Services.SpaceCenter.Vessel active_vessel)
        {
            var flight = active_vessel.Flight();
            StaticPressure = conn.AddStream(() => flight.StaticPressure);
            StaticAirTemperature = conn.AddStream(() => flight.StaticAirTemperature);
            Altitude = conn.AddStream(() => flight.MeanAltitude);

            var orbit = active_vessel.Orbit;
            Apoapsis = conn.AddStream(() => orbit.Apoapsis);
            Periapsis = conn.AddStream(() => orbit.Periapsis);
            TimeToApoapsis = conn.AddStream(() => orbit.TimeToApoapsis);

            var body = orbit.Body;
            GravitationalParameter = conn.AddStream(() => body.GravitationalParameter);
            Radius = conn.AddStream(() => body.EquatorialRadius);

            AvailableThrust = conn.AddStream(() => active_vessel.AvailableThrust);
            Mass = conn.AddStream(() => active_vessel.Mass);
            Velocity = conn.AddStream(() => active_vessel.Velocity(body.ReferenceFrame));
            Position = conn.AddStream(() => active_vessel.Position(body.ReferenceFrame));
        }

        public Stream<float> GravitationalParameter { get; }
        public Stream<float> Radius { get; }
        public Stream<float> StaticPressure { get; }
        public Stream<float> StaticAirTemperature { get; }
        public Stream<double> Apoapsis { get; }
        public Stream<double> Periapsis { get; }
        public Stream<double> TimeToApoapsis { get; }
        public Stream<double> Altitude { get; }
        public Stream<float> AvailableThrust { get; }
        public Stream<float> Mass { get; }
        public Stream<Tuple<double, double, double>> Velocity { get; }
        public Stream<Tuple<double, double, double>> Position { get; }
    }

    public class VesselData
    {
        public void Update()
        {
            Body.GravitationalParameter = Streams.GravitationalParameter.Get();
            Body.Radius = Streams.Radius.Get();
            Environment.StaticPressure = Streams.StaticPressure.Get();
            Environment.Temperature = Streams.StaticAirTemperature.Get();
            Orbit.Apoapsis = Streams.Apoapsis.Get();
            Orbit.Periapsis = Streams.Periapsis.Get();
            Orbit.TimeToApoapsis = Streams.TimeToApoapsis.Get();
            Vessel.Altitude = Streams.Altitude.Get();
            Vessel.AvailableThrust = Streams.AvailableThrust.Get();
            Vessel.Mass = Streams.Mass.Get();
            var velocity = Streams.Velocity.Get();
            Vessel.Velocity = new Vector3d(velocity);
            Vessel.VelocityMag = Vessel.Velocity.Length();
            var position = Streams.Position.Get();
            Vessel.Position = new Vector3d(position);
        }

        public VesselData(Connection conn, KRPC.Client.Services.SpaceCenter.Vessel active_vessel)
        {
            Streams = new VesselStreams(conn, active_vessel);
            Body = new Body();
            Environment = new Environment();
            Orbit = new Orbit();
            Vessel = new Vessel();
        }

        private VesselStreams Streams { get; }
        public Body Body { get; private set; }
        public Environment Environment { get; private set; }
        public Orbit Orbit { get; private set; }
        public Vessel Vessel { get; private set; }
    }
}

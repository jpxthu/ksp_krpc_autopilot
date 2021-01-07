using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public class VesselData
    {
        public class Streams
        {
            public Streams(string vessel_name, Connection conn, Service sc, Vessel active_vessel)
            {
                vessel_name_ = vessel_name;
                Init(conn, sc, active_vessel);
            }

            ~Streams()
            {
                Console.WriteLine("{0}: state streams stopped.", vessel_name_);
            }

            public void Dispose()
            {
                Deinit();
            }

            public void Init(Connection conn, Service sc, Vessel active_vessel)
            {
                var flight = active_vessel.Flight();
                var orbit = active_vessel.Orbit;
                var body = orbit.Body;

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
                AvailablePosTorque = conn.AddStream(() => active_vessel.AvailablePosTorque);
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

            public void Deinit()
            {
                // Environment
                StaticAirTemperature.Remove();
                StaticPressure.Remove();
                // Orbit
                Apoapsis.Remove();
                Periapsis.Remove();
                TimeToApoapsis.Remove();
                // Vessel
                Altitude.Remove();
                AngularVelocity.Remove();
                AvailablePosTorque.Remove();
                AvailableRCSForce.Remove();
                AvailableThrust.Remove();
                AvailableTorque.Remove();
                Direction.Remove();
                DryMass.Remove();
                Forward.Remove();
                Mass.Remove();
                MaxVacuumThrust.Remove();
                MomentOfInertia.Remove();
                Position.Remove();
                Right.Remove();
                Thrust.Remove();
                Up.Remove();
                VacuumSpecificImpulse.Remove();
                Velocity.Remove();
            }

            private readonly string vessel_name_;

            // Environment
            public Stream<float> StaticAirTemperature { get; private set; }
            public Stream<float> StaticPressure { get; private set; }
            // Orbit
            public Stream<double> Apoapsis { get; private set; }
            public Stream<double> Periapsis { get; private set; }
            public Stream<double> TimeToApoapsis { get; private set; }
            // Vessel
            public Stream<double> Altitude { get; private set; }
            public Stream<Tuple<double, double, double>> AngularVelocity { get; private set; }
            public Stream<Tuple<Tuple<double, double, double>, Tuple<double, double, double>>> AvailablePosTorque { get; private set; }
            public Stream<Tuple<Tuple<double, double, double>, Tuple<double, double, double>>> AvailableRCSForce { get; private set; }
            public Stream<float> AvailableThrust { get; private set; }
            public Stream<Tuple<Tuple<double, double, double>, Tuple<double, double, double>>> AvailableTorque { get; private set; }
            public Stream<Tuple<double, double, double>> Direction { get; private set; }
            public Stream<float> DryMass { get; private set; }
            public Stream<Tuple<double, double, double>> Forward { get; private set; }
            public Stream<float> Mass { get; private set; }
            public Stream<float> MaxVacuumThrust { get; private set; }
            public Stream<Tuple<double, double, double>> MomentOfInertia { get; private set; }
            public Stream<Tuple<double, double, double>> Position { get; private set; }
            public Stream<Tuple<double, double, double>> Right { get; private set; }
            public Stream<float> Thrust { get; private set; }
            public Stream<Tuple<double, double, double>> Up { get; private set; }
            public Stream<float> VacuumSpecificImpulse { get; private set; }
            public Stream<Tuple<double, double, double>> Velocity { get; private set; }
        }

        public bool Update()
        {
            try
            {
                // Environment
                Environment.Temperature = streams_.StaticAirTemperature.Get();
                Environment.StaticPressure = streams_.StaticPressure.Get();

                // Orbit
                Orbit.Apoapsis = streams_.Apoapsis.Get();
                Orbit.Periapsis = streams_.Periapsis.Get();
                Orbit.TimeToApoapsis = streams_.TimeToApoapsis.Get();

                // Vessel
                Vessel.Altitude = streams_.Altitude.Get();
                Vessel.AngularVelocity = new Vector3d(streams_.AngularVelocity.Get());
                Vessel.AvailablePosTorque = new Vector3d(streams_.AvailablePosTorque.Get().Item1);
                Vessel.AvailableRCSForce = new TupleV3d(streams_.AvailableRCSForce.Get());
                Vessel.AvailableThrust = streams_.AvailableThrust.Get();
                Vessel.AvailableTorque = new TupleV3d(streams_.AvailableTorque.Get());
                Vessel.Direction = new Vector3d(streams_.Direction.Get());
                Vessel.DryMass = streams_.DryMass.Get();
                Vessel.Forward = new Vector3d(streams_.Forward.Get());
                Vessel.Mass = streams_.Mass.Get();
                Vessel.MaxVacuumThrust = streams_.MaxVacuumThrust.Get();
                Vessel.MomentOfInertia = new Vector3d(streams_.MomentOfInertia.Get());
                Vessel.Position = new Vector3d(streams_.Position.Get());
                Vessel.Right = new Vector3d(streams_.Right.Get());
                Vessel.Thrust = streams_.Thrust.Get();
                Vessel.Up = new Vector3d(streams_.Up.Get());
                Vessel.VacuumSpecificImpulse = streams_.VacuumSpecificImpulse.Get();
                Vessel.Velocity = new Vector3d(streams_.Velocity.Get());
            }
            catch (Exception e)
            {
                Console.WriteLine(
                    "Error when update states. Vessel <{0}> may out of distance or crashed. " +
                    "Error message: {1}", Vessel.Name, e.Message);
                streams_.Dispose();
                Available = false;
                return false;
            }
            
            Vessel.Gravity = common_data_.Body.GravitationalParameter / Vessel.Position.LengthSquared();
            Vessel.MaxFuelRate = Vessel.MaxVacuumThrust / Vessel.VacuumSpecificImpulse / Constants.Common.GRAVITY_ON_KERBIN;
            Vessel.SurfUp = Vessel.Position.Norm();
            Vessel.SurfEast = Vector3d.Cross(Vessel.SurfUp, new Vector3d(0d, 1d, 0d)).Norm();
            Vessel.SurfNorth = Vector3d.Cross(Vessel.SurfEast, Vessel.SurfUp);
            Vessel.VelocityHorizon = Vessel.Velocity - Vessel.Velocity * Vessel.SurfUp * Vessel.SurfUp;
            Vessel.VelocityHorizonMag = Vessel.VelocityHorizon.Length();
            Vessel.VelocityMag = Vessel.Velocity.Length();
            Vessel.VelocityUp = Vessel.Velocity * Vessel.SurfUp;

            Available = true;
            return true;
        }

        public VesselData(string vessel_name, CommonData common_data, Connection conn, Service sc, Vessel vessel)
        {
            vessel_name_ = vessel_name;

            common_data_ = common_data;
            streams_ = new Streams(vessel_name, conn, sc, vessel);
            Environment = new Data.Environment();
            Orbit = new Data.Orbit();
            Vessel = new Data.Vessel(vessel_name);
            Available = false;
        }

        ~VesselData()
        {
            Console.WriteLine("{0}: state updates stopped.", vessel_name_);
        }

        public void Dispose()
        {
            streams_.Dispose();
        }

        private readonly string vessel_name_;

        private readonly CommonData common_data_;
        private readonly Streams streams_;

        public Data.Environment Environment { get; private set; }
        public Data.Orbit Orbit { get; private set; }
        public Data.Vessel Vessel { get; private set; }
        public bool Available { get; private set; }
    }
}

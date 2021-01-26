using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public void Engage()
        {
        }

        public void DisEngage(bool reset = true)
        {
            if (reset)
            {
                Command.SetRcsForward(0d);
                Command.SetRcsRight(0d);
                Command.SetRcsUp(0d);
                Command.SetThrottle(0d);
                Command.ReleaseTargetDirection();
                Execute();
            }
        }

        public Status Execute()
        {
            try
            {
                ActiveVessel.Control.Throttle = Convert.ToSingle(Command.Throttle);
                ActiveVessel.Control.Forward = Convert.ToSingle(Command.RcsForward);
                ActiveVessel.Control.Right = Convert.ToSingle(Command.RcsRight);
                ActiveVessel.Control.Up = Convert.ToSingle(Command.RcsUp);
            }
            catch (Exception e)
            {
                Console.WriteLine(
                    "Error when executing. Vessel <{0}> may out of distance or crashed. Error message: {1}",
                    VesselName, e.Message);
                return Status.FAIL;
            }

            return AtitudeControl();
        }

        public bool UpdateData()
        {
            return State.Update();
        }

        public Control(
            string vessel_name,
            CommonData common_data,
            Connection conn,
            Service sc,
            Vessel vessel)
        {
            Conn = conn;
            ActiveVessel = vessel;
            OrbitBody = vessel.Orbit.Body;
            OrbitBodyFrame = OrbitBody.ReferenceFrame;
            SpaceCenter = conn.SpaceCenter();

            CommonData = common_data;
            State = new VesselData(vessel_name, common_data, conn, sc, vessel);

            Command = new Command();
            Trajectory = new Trajectory(
                vessel_name, common_data, State,
                conn, SpaceCenter, OrbitBody, vessel, 0.0, 100,
                LandingSimThrust);

            VesselName = vessel_name;

            AltitudeControllerInit();
        }

        public void Dispose()
        {
            Trajectory.CalculateStop();

            State.Dispose();
        }

        ~Control()
        {
            Console.WriteLine("{0}: control stopped.", VesselName);
        }

        //private Service SpaceCenter { get; }
        private Vessel ActiveVessel { get; }
        private Connection Conn { get; }
        private CelestialBody OrbitBody { get; }
        private ReferenceFrame OrbitBodyFrame { get; }
        private Service SpaceCenter { get; }

        private CommonData CommonData { get; }
        public VesselData State { get; }

        public Command Command { get; }
        public Trajectory Trajectory { get; }

        public string VesselName { get; }
    }
}

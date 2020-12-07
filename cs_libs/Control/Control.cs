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
                Execute();
            }
        }

        public void Execute()
        {
            ActiveVessel.Control.Throttle = Convert.ToSingle(Command.Throttle);
            ActiveVessel.Control.Forward = Convert.ToSingle(Command.RcsForward);
            ActiveVessel.Control.Right = Convert.ToSingle(Command.RcsRight);
            ActiveVessel.Control.Up = Convert.ToSingle(Command.RcsUp);

            AltitudeControl();
        }

        public void UpdateData()
        {
            State.Update();
        }

        public Control(
            Connection conn,
            Service sc,
            Data.CommonData data,
            Vessel vessel)
        {
            Conn = conn;
            ActiveVessel = vessel;
            OrbitBody = vessel.Orbit.Body;
            SpaceCenter = conn.SpaceCenter();
            Data = data;
            State = new Data.VesselData(conn, sc, vessel);
            Command = new Command();
            Trajectory = new Trajectory(
                data, State,
                conn, SpaceCenter, OrbitBody, vessel, 0.0, 100,
                LandingSimThrust);

            AltitudeControllerInit();
        }

        //private Service SpaceCenter { get; }
        private Connection Conn { get; }
        private Vessel ActiveVessel { get; }
        private CelestialBody OrbitBody { get; }
        private Service SpaceCenter { get; }
        private Data.CommonData Data { get; }
        public Data.VesselData State { get; }
        public Command Command { get; }
        public Trajectory Trajectory { get; }
    }
}

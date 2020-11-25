using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public void Engage()
        {
            Command.Engage();
        }

        public void DisEngage()
        {
            Command.DisEngage();
        }

        public void Execute()
        {
            Command.Execute();
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
            Command = new Command(sc, vessel);
            Trajectory = new Trajectory(
                data, State,
                conn, SpaceCenter, OrbitBody, vessel, 0.0, 100,
                LandingSimThrust);
        }

        //private Service SpaceCenter { get; }
        private Connection Conn { get; }
        private Vessel ActiveVessel { get; }
        private CelestialBody OrbitBody { get; }
        private Service SpaceCenter { get; }
        private Data.CommonData Data { get; }
        private Data.VesselData State { get; }
        private Command Command { get; }
        private Trajectory Trajectory { get; }
    }
}

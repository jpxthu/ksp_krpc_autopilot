using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;

namespace KRPCLibs
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
            Data.CommonData data,
            Vessel vessel)
        {
            ActiveVessel = vessel;
            Data = data;
            State = new Data.VesselData(conn, vessel);
            Command = new Command(vessel);
        }

        //private Service SpaceCenter { get; }
        private Vessel ActiveVessel { get; }
        private Data.CommonData Data { get; }
        private Data.VesselData State { get; }
        private Command Command { get; }
    }
}

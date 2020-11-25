using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Schema.KRPC;
using KrpcAutoPilot.Utils;
using System;
using System.IO;
using System.Net;
using System.Reflection.Metadata;
using System.Threading;

namespace Test
{
    class Program
    {
        static void Main(string[] args)
        {
            var conn = new Connection(
                name: "My Example Program",
                address: IPAddress.Parse("127.0.0.1"),
                rpcPort: 50000,
                streamPort: 50001);

            var sc = conn.SpaceCenter();
            var vessel = sc.ActiveVessel;
            var orbit = vessel.Orbit;
            var body = orbit.Body;

            KrpcAutoPilot.Data.CommonData data = new KrpcAutoPilot.Data.CommonData(conn, sc);

            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(conn, sc, data, vessel);

            var auto_pilot = vessel.AutoPilot;

            while(true)
            {
                Console.WriteLine("{0:0.0}",
                    vessel.AvailableThrust);
                Thread.Sleep(100);
            }

            control.Engage();
            auto_pilot.Engage();
            vessel.Control.RCS = true;

            double tar_height = 20d;
            //double tar_height = 210d;
            Vector3d tar_pos = new Vector3d(body.PositionAtAltitude(
                //KrpcLibs.Constants.Position.KERBAL_CENTER_LAUNCH_PAD_NORTH.Lat,
                //KrpcLibs.Constants.Position.KERBAL_CENTER_LAUNCH_PAD_NORTH.Lng,
                //KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lat,
                //KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lng,
                KrpcAutoPilot.Constants.Position.KERBAL_SEA_LANDING_OFCOUSE_I_LOVE_U.Lat,
                KrpcAutoPilot.Constants.Position.KERBAL_SEA_LANDING_OFCOUSE_I_LOVE_U.Lng,
                tar_height, body.ReferenceFrame));

            Console.WriteLine("Landing init");
            control.LandingInit(tar_height);

            Console.WriteLine("Adjust landing position");
            while (true)
            {
                data.Update();
                control.UpdateData();
                if (control.AdjustLandingPosition(tar_pos))
                {
                    break;
                }
                control.Execute();
                Thread.Sleep(100);
            }

            while (true)
            {
                data.Update();
                control.UpdateData();
                control.Landing1(tar_pos, tar_height);
                control.Execute();
                Thread.Sleep(100);
            }

            vessel.Control.Throttle = 0;
            control.DisEngage();

            conn.Dispose();
        }
    }
}

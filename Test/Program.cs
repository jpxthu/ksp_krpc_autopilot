using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Client.Services.UI;
using KrpcAutoPilot;
using KrpcAutoPilot.Utils;
using System;
using System.Net;
using System.Threading;
using Service = KRPC.Client.Services.SpaceCenter.Service;

namespace Test
{
    class Program
    {
        static void Main(string[] args)
        {
            /*Connection conn = new Connection(
                name: "My Example Program",
                address: IPAddress.Parse("127.0.0.1"),
                rpcPort: 50000,
                streamPort: 50001);
            Service sc = conn.SpaceCenter();
            Vessel vessel = sc.ActiveVessel;
            Orbit orbit = vessel.Orbit;
            CelestialBody body = orbit.Body;

            var common_data = new CommonData(conn, sc, body);
            Thread common_data_thread = new Thread(() =>
            {
                while (true)
                {
                    if (!common_data.Update())
                        break;
                    Thread.Sleep(100);
                }
            });
            common_data_thread.Start();

            KrpcAutoPilot.Control control = new KrpcAutoPilot.Control("1", common_data, conn, sc, vessel);

            while (true)
            {
                control.UpdateData();
                control.Command.SetTargetDirection(new Vector3d(vessel.Position(body.ReferenceFrame)).Norm());
                control.Command.SetHeadingAngle(0d);
                control.Execute();
                Thread.Sleep(100);
            }*/

            FH fh = new FH();
            fh.Start(FH.FocusPart.BOOSTER_NORTH);

            //Docking docking = new Docking();
            //docking.Start("Kerbin空间站", "docking_port_2", "docking_port", 50);

            //Console.ReadKey();

            //Recycle recycle = new Recycle();
            //recycle.Start(80d, 2d);
        }
    }
}

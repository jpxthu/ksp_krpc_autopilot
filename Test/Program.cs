using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Schema.KRPC;
using System;
using System.IO;
using System.Net;
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

            var drawing = conn.Drawing();

            var sc = conn.SpaceCenter();
            var vessel = sc.ActiveVessel;
            var orbit = vessel.Orbit;
            var body = orbit.Body;
            var flight = vessel.Flight();

            KRPCLibs.Data.CommonData data = new KRPCLibs.Data.CommonData(conn, sc);
            KRPCLibs.Data.VesselData vdata = new KRPCLibs.Data.VesselData(conn, vessel);

            Thruster t = vessel.Parts.Engines[0].Thrusters[0];

            using (StreamWriter sw = new StreamWriter("a.tsv")) {
                while (true) {
                    var v = vessel.Velocity(body.ReferenceFrame);
                    var d = vessel.Flight().Drag;
                    string s = string.Format("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}",
                        v.Item1, v.Item2, v.Item3,
                        d.Item1, d.Item2, d.Item3,
                        vessel.Flight().AtmosphereDensity);
                    sw.WriteLine(s);
                }
            }

            while (true) {
                vdata.Update();
                var v = KRPCLibs.Utils.Trajectory.ImpactPosition(vdata, 0.0, 6.0);
                var p = new System.Tuple<double, double, double>(v.X, v.Y, v.Z);
                var la = body.LatitudeAtPosition(p, body.ReferenceFrame);
                var lo = body.LongitudeAtPosition(p, body.ReferenceFrame);
                Thread.Sleep(100);
            }

            return;

            KRPCLibs.Control control = new KRPCLibs.Control(conn, data, vessel);

            control.Engage();

            while (true) {
                data.Update();
                control.UpdateData();
                if (control.LaunchIntoApoapsis(80000)) {
                    break;
                }
                control.Execute();
                Thread.Sleep(100);
            }

            control.LaunchIntoPeriapsisInit();

            while (true) {
                data.Update();
                control.UpdateData();
                if (control.LaunchIntoPeriapsis(80000)) {
                    break;
                }
                control.Execute();
                Thread.Sleep(100);
            }

            vessel.Control.Throttle = 0;
            control.DisEngage();

            conn.Dispose();
        }
    }
}

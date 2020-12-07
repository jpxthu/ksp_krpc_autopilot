using CsSamples;
using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.KRPC;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;
using System.Linq;
using System.Net;
using System.Threading;

namespace Test
{
    class Program
    {
        static void Main(string[] args)
        {
            Connection conn = new Connection(
                name: "My Example Program",
                address: IPAddress.Parse("127.0.0.1"),
                rpcPort: 50000,
                streamPort: 50001);

            var sc = conn.SpaceCenter();
            Vessel vessel = sc.ActiveVessel;
            Orbit orbit = vessel.Orbit;
            CelestialBody body = orbit.Body;
            KrpcAutoPilot.Data.CommonData data = new KrpcAutoPilot.Data.CommonData(conn, sc);

            //conn.Drawing().AddDirection(new Tuple<double, double, double>(1, 0, 0), vessel.ReferenceFrame, 10f);  // pitch  right
            //conn.Drawing().AddDirection(new Tuple<double, double, double>(0, 1, 0), vessel.ReferenceFrame, 20f);  // roll   forward
            //conn.Drawing().AddDirection(new Tuple<double, double, double>(0, 0, 1), vessel.ReferenceFrame, 30f);  // yaw    down

            /*KrpcAutoPilot.Control control = new KrpcAutoPilot.Control(conn, sc, data, vessel);
            while (true)
            {
                data.Update();
                control.UpdateData();
                control.Command.SetTargetDirection(1, 0, 0);
                control.Command.SetHeading(0);
                control.Execute();
                Thread.Sleep(100);
            }*/

            bool should_exit = false;
            Thread thread_main = new Thread(() =>
            {
                while (!should_exit)
                {
                    data.Update();
                    Thread.Sleep(100);
                }
            });
            thread_main.Start();

            // Parts
            Engine engine_main = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_main").ToList().First();
            Engine engine_north = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_north").ToList().First();
            Engine engine_south = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_south").ToList().First();
            Part tank_north = vessel.Parts.WithTag("tank_north").ToList().First();
            Part tank_south = vessel.Parts.WithTag("tank_south").ToList().First();

            // Launch
            engine_main.ThrustLimit = 0.6f;
            //engine_main.GimbalLimit = 0.2f;
            //engine_north.GimbalLimit = 0.2f;
            //engine_south.GimbalLimit = 0.2f;
            Thread launch_thread = new Thread(() => VesselControl.Launch(conn, sc, vessel, data, 80000));
            launch_thread.Start();

            // Splite
            float north_fuel_throshold = 3000f;// tank_south.Resources.Max("LiquidFuel");
            var north_fuel = Connection.GetCall(() => tank_south.Resources.Amount("LiquidFuel"));
            var north_split_expr = Expression.LessThan(conn,
                Expression.Call(conn, north_fuel),
                Expression.ConstantFloat(conn, north_fuel_throshold));
            var north_split_event = conn.KRPC().AddEvent(north_split_expr);
            lock (north_split_event.Condition)
                north_split_event.Wait();
            vessel.Control.ActivateNextStage();

            Vessel vessel_north = engine_north.Part.Vessel;
            Vessel vessel_south = engine_south.Part.Vessel;
            sc.ActiveVessel = vessel_south;
            vessel_north.Control.Throttle = 0.1f;
            vessel_north.Control.RCS = true;
            vessel_north.Control.Right = -1f;
            vessel_north.Control.Up = -1f;
            vessel_south.Control.Throttle = 0.1f;
            vessel_south.Control.RCS = true;
            vessel_south.Control.Right = 1f;
            vessel_south.Control.Up = -1f;
            Thread.Sleep(300);
            vessel_north.Control.Throttle = 0f;
            vessel_north.Control.Right = 0f;
            vessel_south.Control.Throttle = 0f;
            vessel_south.Control.Right = 0f;
            Thread.Sleep(1700);
            vessel_north.Control.Up = 0f;
            vessel_south.Control.Up = 0f;
            engine_north.GimbalLimit = 1f;
            engine_south.GimbalLimit = 1f;

            // Recycle
            double tar_altitude = 205d;
            Thread recycle_north_thread = new Thread(() => VesselControl.Recycle(
                conn, sc, vessel_north, KrpcAutoPilot.Control.RcsLayout.TOP, data,
                new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lat,
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude));
            recycle_north_thread.Start();
            Thread recycle_south_thread = new Thread(() => VesselControl.Recycle(
                conn, sc, vessel_south, KrpcAutoPilot.Control.RcsLayout.TOP, data,
                new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lat,
                    KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude));
            recycle_south_thread.Start();

            recycle_north_thread.Join();
            recycle_south_thread.Join();
            should_exit = true;

            conn.Dispose();
        }
    }
}

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

using LandingAdjustBurnStatus = KrpcAutoPilot.Control.LandingAdjustBurnStatus;

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
                control.Command.SetTargetDirection(0, 0, -1);
                control.Command.SetHeading(180);
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
            Part tank_main = vessel.Parts.WithTag("tank_main").ToList().First();

            // Launch
            engine_main.ThrustLimit = 0.6f;
            Thread launch_thread = new Thread(() => VesselControl.Launch(conn, sc, vessel, data, 80000));
            launch_thread.Start();

            // Splite booster
            float booster_fuel_throshold = 3000f;// tank_south.Resources.Max("LiquidFuel");
            var north_fuel = Connection.GetCall(() => tank_south.Resources.Amount("LiquidFuel"));
            var north_split_expr = Expression.LessThan(conn,
                Expression.Call(conn, north_fuel),
                Expression.ConstantFloat(conn, booster_fuel_throshold));
            var north_split_event = conn.KRPC().AddEvent(north_split_expr);
            lock (north_split_event.Condition)
                north_split_event.Wait();
            engine_main.ThrustLimit = 1f;
            engine_north.ThrustLimit = 0f;
            engine_south.ThrustLimit = 0f;
            Thread.Sleep(1000);
            vessel.Control.ActivateNextStage();

            Vessel vessel_north = engine_north.Part.Vessel;
            Vessel vessel_south = engine_south.Part.Vessel;
            sc.ActiveVessel = vessel_north;
            vessel_north.Control.Throttle = 0f;
            vessel_north.Control.RCS = true;
            //vessel_north.Control.Right = -1f;
            vessel_north.Control.Up = 1f;
            vessel_south.Control.Throttle = 0f;
            vessel_south.Control.RCS = true;
            //vessel_south.Control.Right = -1f;
            vessel_south.Control.Up = -1f;
            Thread.Sleep(500);
            //vessel_north.Control.Throttle = 0f;
            //vessel_north.Control.Right = 0f;
            //vessel_south.Control.Throttle = 0f;
            //vessel_south.Control.Right = 0f;
            //Thread.Sleep(1500);
            vessel_north.Control.Up = 0f;
            vessel_south.Control.Up = 0f;
            engine_north.ThrustLimit = 1f;
            engine_south.ThrustLimit = 1f;

            // Recycle booster
            double tar_altitude = 205d;
            LandingAdjustBurnStatus landingAdjustBurnStatusNorth = LandingAdjustBurnStatus.UNAVAILABEL;
            LandingAdjustBurnStatus landingAdjustBurnStatusSouth = LandingAdjustBurnStatus.UNAVAILABEL;
            bool landingAdjustBurn = false;
            Thread recycle_north_thread = new Thread(() => VesselControl.Recycle(
                conn, sc, vessel_north, KrpcAutoPilot.Control.RcsLayout.TOP, data,
                new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lat,
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude,
                0d,
                ref landingAdjustBurnStatusNorth,
                ref landingAdjustBurn));
            Thread recycle_south_thread = new Thread(() => VesselControl.Recycle(
                conn, sc, vessel_south, KrpcAutoPilot.Control.RcsLayout.TOP, data,
                new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lat,
                    KrpcAutoPilot.Constants.Position.VAB_TOP_WEST.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude,
                Math.PI,
                ref landingAdjustBurnStatusSouth,
                ref landingAdjustBurn));
            recycle_north_thread.Start();
            recycle_south_thread.Start();

            new Thread(o =>
            {
                while (true)
                {
                    Thread.Sleep(100);
                    if (landingAdjustBurnStatusNorth == LandingAdjustBurnStatus.WAITING &&
                        landingAdjustBurnStatusSouth == LandingAdjustBurnStatus.WAITING)
                        break;
                }
                landingAdjustBurn = true;
            }).Start();

            // Recycle first stage
            Thread recycle_main_thread = null;
            if (NeedRecycleMainVessel(conn, tank_main))
            {
                vessel.Control.Throttle = 0f;
                Thread.Sleep(1000);
                vessel.Control.ActivateNextStage();

                Vessel vessel_main = engine_main.Part.Vessel;
                recycle_main_thread = new Thread(() => VesselControl.Recycle(
                    conn, sc, vessel_main, KrpcAutoPilot.Control.RcsLayout.TOP, data,
                    new Vector3d(sc.Vessels.Where(v => v.Name == "landing_ship").First().Position(body.ReferenceFrame)),
                    20d,
                    0d,
                    ref landingAdjustBurnStatusNorth,
                    ref landingAdjustBurn));
                recycle_main_thread.Start();
            }

            recycle_north_thread.Join();
            recycle_south_thread.Join();
            if (!(recycle_main_thread is null))
                recycle_main_thread.Join();
            should_exit = true;

            conn.Dispose();
        }

        static bool NeedRecycleMainVessel(Connection conn, Part tank)
        {
            float main_throshold = 2000f;
            Stream<float> fuel_stream = conn.AddStream(() => tank.Resources.Amount("LiquidFuel"));
            try
            {
                while (true)
                {
                    float fuel = fuel_stream.Get();
                    if (fuel <= main_throshold)
                        return true;
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                return false;
            }
        }
    }
}

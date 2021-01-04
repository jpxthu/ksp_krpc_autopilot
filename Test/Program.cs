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

            KrpcAutoPilot.CommonData common_data = new KrpcAutoPilot.CommonData(conn, sc, body);
            bool stop = false;
            Thread common_data_thread = new Thread(() =>
            {
                while (!stop)
                {
                    common_data.Update();
                    Thread.Sleep(100);
                }
                common_data.Dispose();
            });
            common_data_thread.Start();

            // Parts
            Engine engine_load = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_load").ToList().First();
            Engine engine_main = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_main").ToList().First();
            Engine engine_north = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_north").ToList().First();
            Engine engine_south = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_south").ToList().First();
            Part tank_north = vessel.Parts.WithTag("tank_north").ToList().First();
            Part tank_south = vessel.Parts.WithTag("tank_south").ToList().First();
            Part tank_main = vessel.Parts.WithTag("tank_main").ToList().First();

            // Launch
            engine_main.ThrustLimit = 0.6f;
            Thread launch_thread = new Thread(() => VesselControl.Launch("LOAD", common_data, conn, sc, vessel, 80000));
            launch_thread.Start();

            // Splite booster
            WaitTankFuelLessThanThreshold(conn, tank_south, 2500f);
            engine_main.ThrustLimit = 1f;
            engine_north.ThrustLimit = 0f;
            engine_south.ThrustLimit = 0f;
            Thread.Sleep(1000);
            vessel.Control.ActivateNextStage();

            Vessel vessel_north = engine_north.Part.Vessel;
            Vessel vessel_south = engine_south.Part.Vessel;
            //sc.ActiveVessel = vessel_south;
            vessel_north.Control.Throttle = 0f;
            vessel_south.Control.Throttle = 0f;
            engine_north.ThrustLimit = 1f;
            engine_south.ThrustLimit = 1f;

            // Recycle booster
            double tar_altitude = 205d;
            LandingAdjustBurnStatus landingAdjustBurnStatusNorth = LandingAdjustBurnStatus.UNAVAILABEL;
            LandingAdjustBurnStatus landingAdjustBurnStatusSouth = LandingAdjustBurnStatus.UNAVAILABEL;
            bool landingAdjustBurn = false;
            Thread recycle_north_thread = new Thread(() => VesselControl.Recycle(
                "NORTH", common_data, conn, sc, vessel_north, KrpcAutoPilot.Control.RcsLayout.TOP,
                new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lat,
                    KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude,
                0d,
                ref landingAdjustBurnStatusNorth,
                ref landingAdjustBurn));
            Thread recycle_south_thread = new Thread(() => VesselControl.Recycle(
                "SOUTH", common_data, conn, sc, vessel_south, KrpcAutoPilot.Control.RcsLayout.TOP,
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
            if (WaitTankFuelLessThanThreshold(conn, tank_main, 3000f))
                engine_load.ThrustLimit = 0f;
            engine_main.ThrustLimit = 0f;
            Thread.Sleep(1000);
            vessel.Control.ActivateNextStage();

            Vessel vessel_main = engine_main.Part.Vessel;
            sc.ActiveVessel = vessel_main;
            vessel_main.Control.RCS = true;
            vessel_main.Control.Throttle = 0f;
            vessel_main.Control.Forward = -1f;
            Thread.Sleep(1000);
            vessel_main.Control.Forward = 0f;
            engine_load.ThrustLimit = 1f;
            engine_main.ThrustLimit = 1f;

            Thread recycle_main_thread = new Thread(() =>
            {
                LandingAdjustBurnStatus landingAdjustBurnStatusMain = LandingAdjustBurnStatus.UNAVAILABEL;
                bool landingAdjustBurnMain = true;
                VesselControl.Recycle(
                    "MAIN", common_data, conn, sc, vessel_main, KrpcAutoPilot.Control.RcsLayout.TOP,
                    new Vector3d(sc.Vessels.Where(v => v.Name == "landing_ship").First().Position(body.ReferenceFrame)),
                    20d,
                    0d,
                    ref landingAdjustBurnStatusMain,
                    ref landingAdjustBurnMain);
            });
            recycle_main_thread.Start();

            launch_thread.Join();
            recycle_north_thread.Join();
            recycle_south_thread.Join();
            recycle_main_thread.Join();

            stop = true;
            common_data_thread.Join();

            conn.Dispose();
        }

        static bool WaitTankFuelLessThanThreshold(Connection conn, Part tank, float threshold)
        {
            Stream<float> fuel_stream;
            try
            {
                fuel_stream = conn.AddStream(() => tank.Resources.Amount("LiquidFuel"));
            }
            catch(Exception)
            {
                return false;
            }

            try
            {
                while (true)
                {
                    float fuel = fuel_stream.Get();
                    if (fuel <= threshold)
                    {
                        fuel_stream.Remove();
                        return true;
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
                fuel_stream.Remove();
                return false;
            }
        }
    }
}

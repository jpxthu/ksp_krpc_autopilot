using CsSamples;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Threading;

using LandingAdjustBurnStatus = KrpcAutoPilot.Control.LandingAdjustBurnStatus;

class FH
{
    public enum FocusPart
    {
        BOOSTER_NORTH,
        BOOSTER_SOUTH,
        FIRST_STAGE,
        LOAD
    }

    private const float BOOSTER_SPLIT_FUEL = 2500;
    private const float FIRST_STAGE_SPLIT_FUEL = 3000;

    private Connection conn;
    private Service sc;
    private Vessel vessel, vessel_north, vessel_south, vessel_main;
    private Orbit orbit;
    private CelestialBody body;
    private KrpcAutoPilot.CommonData common_data;

    // Parts
    private Engine engine_load, engine_main, engine_north, engine_south;
    private Part tank_north, tank_south, tank_main;

    private bool running;

    private bool UpdateCommonData(out Thread common_data_thread)
    {
        common_data = new KrpcAutoPilot.CommonData(conn, sc, body);
        common_data_thread = new Thread(() =>
        {
            while (running)
            {
                if (!common_data.Update())
                    break;
                Thread.Sleep(100);
            }
            common_data.Dispose();
        });
        common_data_thread.Start();
        return true;
    }

    private bool Launch(out Thread launch_thread)
    {
        engine_main.ThrustLimit = 0.6f;
        launch_thread = new Thread(() => VesselControl.Launch("LOAD", common_data, conn, sc, vessel, 80000));
        launch_thread.Start();
        return true;
    }

    private bool WaitTankFuelLessThanThreshold(Connection conn, Part tank, float threshold)
    {
        Stream<float> fuel_stream;
        try
        {
            fuel_stream = conn.AddStream(() => tank.Resources.Amount("LiquidFuel"));
        }
        catch (Exception)
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

    private bool SplitBoosters(FocusPart focus_part)
    {
        // Splite booster
        if (!WaitTankFuelLessThanThreshold(conn, tank_south, BOOSTER_SPLIT_FUEL))
            return false;

        try
        {
            engine_main.ThrustLimit = 1f;
            engine_north.ThrustLimit = 0f;
            engine_south.ThrustLimit = 0f;
            Thread.Sleep(1000);
            vessel.Control.ActivateNextStage();

            vessel_north = engine_north.Part.Vessel;
            vessel_south = engine_south.Part.Vessel;

            switch (focus_part)
            {
                case FocusPart.BOOSTER_NORTH: sc.ActiveVessel = vessel_north; break;
                case FocusPart.BOOSTER_SOUTH: sc.ActiveVessel = vessel_south; break;
                default: break;
            }

            vessel_north.Control.Throttle = 0f;
            vessel_south.Control.Throttle = 0f;
            engine_north.ThrustLimit = 1f;
            engine_south.ThrustLimit = 1f;
        }
        catch (Exception)
        {
            return false;
        }

        return true;
    }

    private bool RecycleBoosters(
        out Thread recycle_north_thread,
        out Thread recycle_south_thread)
    {
        LandingAdjustBurnStatus landingAdjustBurnStatusNorth = LandingAdjustBurnStatus.UNAVAILABEL;
        LandingAdjustBurnStatus landingAdjustBurnStatusSouth = LandingAdjustBurnStatus.UNAVAILABEL;
        bool landingAdjustBurn = false;

        double tar_altitude = 215d;
        recycle_north_thread = new Thread(() => VesselControl.Recycle(
            "NORTH", common_data, conn, sc, vessel_north, KrpcAutoPilot.Control.RcsLayout.TOP,
            new Vector3d(body.PositionAtAltitude(
                KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lat,
                KrpcAutoPilot.Constants.Position.VAB_TOP_EAST.Lng,
                tar_altitude, body.ReferenceFrame)),
            tar_altitude,
            0d,
            ref landingAdjustBurnStatusNorth,
            ref landingAdjustBurn));
        recycle_south_thread = new Thread(() => VesselControl.Recycle(
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
                if (landingAdjustBurnStatusNorth == LandingAdjustBurnStatus.ABANDON ||
                    landingAdjustBurnStatusSouth == LandingAdjustBurnStatus.ABANDON)
                    return;
                if (landingAdjustBurnStatusNorth == LandingAdjustBurnStatus.WAITING &&
                    landingAdjustBurnStatusSouth == LandingAdjustBurnStatus.WAITING)
                    break;
            }
            landingAdjustBurn = true;
        }).Start();

        return true;
    }

    private bool SplitFirstStage(FocusPart focus_part)
    {
        if (!WaitTankFuelLessThanThreshold(conn, tank_main, FIRST_STAGE_SPLIT_FUEL))
            return false;

        try
        {
            engine_load.ThrustLimit = 0f;
            engine_main.ThrustLimit = 0f;
            Thread.Sleep(1000);
            vessel.Control.ActivateNextStage();

            vessel_main = engine_main.Part.Vessel;
            if (focus_part == FocusPart.FIRST_STAGE)
                sc.ActiveVessel = vessel_main;
            vessel_main.Control.RCS = true;
            vessel_main.Control.Throttle = 0f;
            vessel_main.Control.Forward = -1f;
            vessel.Control.RCS = true;
            vessel.Control.Forward = 1f;
            Thread.Sleep(1000);
            vessel_main.Control.Forward = 0f;
            vessel.Control.Forward = 0f;
            engine_load.ThrustLimit = 1f;
            engine_main.ThrustLimit = 1f;
        }
        catch (Exception)
        {
            return false;
        }

        return true;
    }

    private bool RecycleFirstStage(out Thread recycle_main_thread)
    {
        recycle_main_thread = new Thread(() =>
        {
            LandingAdjustBurnStatus landingAdjustBurnStatusMain = LandingAdjustBurnStatus.UNAVAILABEL;
            bool landingAdjustBurnMain = true;
            VesselControl.Recycle(
                "MAIN", common_data, conn, sc, vessel_main, KrpcAutoPilot.Control.RcsLayout.TOP,
                new Vector3d(sc.Vessels.Where(v => v.Name == "landing_ship").First().Position(body.ReferenceFrame)),
                30d,
                0d,
                ref landingAdjustBurnStatusMain,
                ref landingAdjustBurnMain);
        });
        recycle_main_thread.Start();
        return true;
    }

    private bool StartThreads(FocusPart focus_part, out List<Thread> threads)
    {
        threads = new List<Thread>();

        // Parts
        engine_load = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_load").ToList().First();
        engine_main = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_main").ToList().First();
        engine_north = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_north").ToList().First();
        engine_south = vessel.Parts.Engines.Where(p => p.Part.Tag == "engine_south").ToList().First();
        tank_north = vessel.Parts.WithTag("tank_north").ToList().First();
        tank_south = vessel.Parts.WithTag("tank_south").ToList().First();
        tank_main = vessel.Parts.WithTag("tank_main").ToList().First();

        if (!Launch(out Thread launch_thread))
            return false;
        threads.Add(launch_thread);

        if (!SplitBoosters(focus_part) ||
            !RecycleBoosters(out Thread recycle_north_thread, out Thread recycle_south_thread))
            return false;
        threads.Add(recycle_north_thread);
        threads.Add(recycle_south_thread);

        if (!SplitFirstStage(focus_part) ||
            !RecycleFirstStage(out Thread recycle_main_thread))
            return false;
        threads.Add(recycle_main_thread);

        return true;
    }

    public void Start(FocusPart focus_part)
    {
        running = true;

        conn = new Connection(
            name: "My Example Program",
            address: IPAddress.Parse("127.0.0.1"),
            rpcPort: 50000,
            streamPort: 50001);
        sc = conn.SpaceCenter();
        vessel = sc.ActiveVessel;
        orbit = vessel.Orbit;
        body = orbit.Body;

        UpdateCommonData(out Thread common_data_thread);
        StartThreads(focus_part, out List<Thread> threads);
        foreach (Thread thread in threads)
            thread.Join();

        running = false;
        common_data_thread.Join();

        conn.Dispose();
    }
}

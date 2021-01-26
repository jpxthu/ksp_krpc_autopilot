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

class Docking
{
    public void Start(
        string vessel_name,
        string port_tag,
        string control_node_tag,
        double distance)
    {
        bool running = true;

        Connection conn = new Connection(
            name: "My Example Program",
            address: IPAddress.Parse("127.0.0.1"),
            rpcPort: 50000,
            streamPort: 50001);
        Service sc = conn.SpaceCenter();
        Vessel vessel = sc.ActiveVessel;
        Orbit orbit = vessel.Orbit;
        CelestialBody body = orbit.Body;

        KrpcAutoPilot.CommonData common_data = new KrpcAutoPilot.CommonData(conn, sc, body);
        Thread common_data_thread = new Thread(() =>
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

        KrpcAutoPilot.Control control = new KrpcAutoPilot.Control("1", common_data, conn, sc, vessel);
        control.UpdateData();
        control.ApproachInit(
            ship_name: vessel_name,
            dock_tag: port_tag,
            control_node_tag: control_node_tag,
            distance: distance);
        Thread approach_thread = new Thread(() =>
        {
            while (true)
            {
                control.UpdateData();
                if (control.Approach(out _) == KrpcAutoPilot.Control.Status.FINISHED)
                    break;
                if (control.Execute() == KrpcAutoPilot.Control.Status.FAIL)
                    break;
                Thread.Sleep(100);
            }
        });
        approach_thread.Start();
        approach_thread.Join();
        control.ApproachDeinit();

        running = false;
        common_data_thread.Join();

        conn.Dispose();
    }
}

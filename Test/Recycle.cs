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

class Recycle
{
    public void Start(
        double tar_altitude,
        double landing_min_velocity)
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
        Thread recycle_thread = new Thread(() =>
        {
            LandingAdjustBurnStatus landingAdjustBurnStatusMain = LandingAdjustBurnStatus.UNAVAILABEL;
            bool landingAdjustBurnMain = true;
            VesselControl.Recycle(
                vessel_name: "MAIN",
                common_data: common_data,
                connection: conn,
                space_center: sc,
                vessel: vessel,
                rcs_layout: KrpcAutoPilot.Control.RcsLayout.SYMMETRICAL,
                tar_pos: new Vector3d(body.PositionAtAltitude(
                    KrpcAutoPilot.Constants.Position.KERBAL_CENTER_LAUNCH_PAD.Lat,
                    KrpcAutoPilot.Constants.Position.KERBAL_CENTER_LAUNCH_PAD.Lng,
                    tar_altitude, body.ReferenceFrame)),
                tar_altitude: tar_altitude,
                landing_min_velocity: landing_min_velocity,
                heading: 0d,
                landing_adjust_burn_status: ref landingAdjustBurnStatusMain,
                landing_adjust_could_burn: ref landingAdjustBurnMain);
        });
        recycle_thread.Start();
        recycle_thread.Join();

        running = false;
        common_data_thread.Join();

        conn.Dispose();
    }
}

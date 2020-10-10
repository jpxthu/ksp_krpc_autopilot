using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Schema.KRPC;
using KrpcLibs.Math;
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

            var sc = conn.SpaceCenter();
            var vessel = sc.ActiveVessel;
            var orbit = vessel.Orbit;
            var body = orbit.Body;

            KrpcLibs.Data.CommonData data = new KrpcLibs.Data.CommonData(conn, sc);

            KrpcLibs.Control control = new KrpcLibs.Control(conn, data, vessel);

            var flight = vessel.Flight(body.ReferenceFrame);
            var auto_pilot = vessel.AutoPilot;

            control.Engage();
            auto_pilot.Engage();
            auto_pilot.TargetPitch = 90f;
            vessel.Control.RCS = true;

            double tar_height = /*flight.MeanAltitude*/ +110d;
            double max_rcs_thrust = vessel.Parts.RCS[0].MaxThrust * 6;

            while (true)
            {
                double max_thrust = vessel.MaxThrust;
                double mass = vessel.Mass;
                double height = flight.MeanAltitude;
                Vector3d up = new Vector3d(vessel.Position(body.ReferenceFrame)).Norm();
                Vector3d vel = new Vector3d(vessel.Velocity(body.ReferenceFrame));
                double vel_up = vel * up;
                double tar_vel = Math.Clamp(tar_height - height, -1000d, 1000d);
                double tar_acc = (tar_vel - vel_up) * 2;
                double tar_thrust = mass * (body.SurfaceGravity + tar_acc);
                vessel.Control.Throttle = Math.Clamp(Convert.ToSingle(tar_thrust / max_thrust), 0f, 1f);

                Vector3d tar_pos_h = new Vector3d(body.PositionAtAltitude(
                    KrpcLibs.Constants.Position.KERBAL_CENTER_LAUNCH_PAD_NORTH_GEO.Lat,
                    KrpcLibs.Constants.Position.KERBAL_CENTER_LAUNCH_PAD_NORTH_GEO.Lng,
                    //-(0.0 + (5.0 / 60.0) + (48.38 / 60.0 / 60.0)),
                    //-(74.0 + (37.0 / 60.0) + (12.2 / 60.0 / 60.0)),
                    tar_height, vessel.ReferenceFrame));
                Vector3d tar_vel_h = Math.Sqrt(2d * max_rcs_thrust / mass * 0.8 * tar_pos_h.Length()) * tar_pos_h.Norm();
                Vector3d vel_h = new Vector3d(sc.TransformDirection(vessel.Flight(body.ReferenceFrame).Velocity, body.ReferenceFrame, vessel.ReferenceFrame));
                Vector3d tar_acc_h = (tar_vel_h - vel_h) * 2d;

                vessel.Control.Right = Math.Clamp(Convert.ToSingle(tar_acc_h.X * mass / max_rcs_thrust), -1f, 1f);
                vessel.Control.Up = Math.Clamp(-Convert.ToSingle(tar_acc_h.Z * mass / max_rcs_thrust), -1f, 1f);

                var res = vessel.Flight(body.ReferenceFrame).SimulateAerodynamicForceAt(
                    body,
                    new Tuple<double, double, double>(0, 0, 0),
                    new Tuple<double, double, double>(up.X, up.Y, up.Z));
                Console.WriteLine("{0}\t{1}\t{2}\t{3}",
                    //Convert.ToInt32(res.Item1),
                    //Convert.ToInt32(res.Item2),
                    //Convert.ToInt32(res.Item3));
                    tar_pos_h.X,
                    tar_pos_h.Z,
                    vel_h.X,
                    vel_h.Z);
                Thread.Sleep(100);
            }

            while (true)
            {
                data.Update();
                control.UpdateData();
                if (control.LaunchIntoApoapsis(80000))
                {
                    break;
                }
                control.Execute();
                Thread.Sleep(100);
            }

            control.LaunchIntoPeriapsisInit();

            while (true)
            {
                data.Update();
                control.UpdateData();
                if (control.LaunchIntoPeriapsis(80000))
                {
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

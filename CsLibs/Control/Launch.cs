//using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool launch_burn_stage = false;
        private double launch_apoapsis;
        private Node launch_periapsis_node;

        public void LaunchIntoPeriapsisInit()
        {
            launch_burn_stage = false;
            launch_apoapsis = State.Orbit.Apoapsis;
        }

        public bool LaunchIntoApoapsis(double tar_apoapsis, double heading, double max_q = 15000d)
        {
            double tar_pitch = Math.Sqrt(Math.Max(0d, 1d - (State.Orbit.Apoapsis - CommonData.Body.Radius) / tar_apoapsis)) * Math.PI / 2d;

            double q = Common.DynamicPressure(
                State.Environment.StaticPressure,
                State.Environment.Temperature,
                State.Vessel.VelocityMag);
            double throttle = Math.Min(1d, Math.Max(0d, (1.1d - q / max_q) * 5d));

            Vector3d dir = State.Vessel.SurfEast * Math.Cos(tar_pitch) + State.Vessel.SurfUp * Math.Sin(tar_pitch);
            Command.SetTargetDirection(dir);
            Command.SetHeadingAngle(heading);
            Command.SetThrottle(throttle);
            //Conn.Drawing().Clear();
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(State.Vessel.East.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame, 20f);
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(State.Vessel.BodyUp.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame, 30f);
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(dir.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame, 40f);

            return State.Orbit.Apoapsis >= tar_apoapsis + CommonData.Body.Radius;
        }

        public bool LaunchIntoPeriapsis(double tar_periapsis)
        {
            tar_periapsis += CommonData.Body.Radius;

            if (launch_burn_stage)
            {
                Command.SetThrottle(1d);
                if (State.Orbit.Apoapsis + State.Orbit.Periapsis >= tar_periapsis + launch_apoapsis)
                {
                    try
                    {
                        launch_periapsis_node.Remove();
                    }
                    catch (Exception)
                    { }
                    return true;
                }
                else
                {
                    return false;
                }
            }

            double max_acc = State.Vessel.AvailableThrust / State.Vessel.Mass;

            double vel_apoapsis = State.Vessel.VelocityMag * (State.Vessel.Altitude + CommonData.Body.Radius) / State.Orbit.Apoapsis;
            double vel_apoapsis_tar = Math.Sqrt(CommonData.Body.GravitationalParameter / State.Orbit.Apoapsis);
            double dvel_tar = vel_apoapsis_tar - vel_apoapsis;
            double burn_time = dvel_tar / max_acc;
            double drift_time = State.Orbit.TimeToApoapsis - burn_time / 2d - 2d;

            double tar_pitch = (1d - (State.Orbit.Apoapsis - CommonData.Body.Radius) / (tar_periapsis - CommonData.Body.Radius)) * Math.PI / 2d;
            Command.SetTargetDirection(State.Vessel.SurfEast * Math.Cos(tar_pitch) + State.Vessel.SurfUp * Math.Sin(tar_pitch));
            Command.SetThrottle(Math.Max(0d, Math.Min(1d, (tar_periapsis - State.Orbit.Apoapsis) / 20d)));

            if (drift_time < 0d)
            {
                try
                {
                    Node node = ActiveVessel.Control.AddNode(CommonData.UT + State.Orbit.TimeToApoapsis, Convert.ToSingle(dvel_tar));
                    //Command.SetTargetDirection(node.Direction(ActiveVessel.ReferenceFrame));
                    ActiveVessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
                    Command.SetTargetDirection(State.Vessel.SurfEast);
                    launch_burn_stage = true;
                    launch_periapsis_node = node;
                }
                catch
                {
                    return true;
                }
            }

            return false;
        }
    }
}

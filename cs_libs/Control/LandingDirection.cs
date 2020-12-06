using KRPC.Client.Services.Drawing;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public bool LandingDirection(Vector3d tar_pos, double tar_altitude)
        {
            if (!Trajectory.ResultAvailable)
            {
                Command.SetTargetDirection(-State.Vessel.Velocity.Norm());
                return false;
            }

            Vector3d tar_pos_v = tar_pos - Trajectory.ImpactPositionWithAction;
            double distance = tar_pos_v.Length();
            double throttle = -1d;

            if (Trajectory.NextBurnTime > 3d)
            {

                if (Trajectory.LiftEstimationForceAve < State.Vessel.AvailableThrust * 0.001d &&
                    (distance > Trajectory.NextBurnTime * 50d + 200d || (distance > 200d && landing_adjust_throttle > 0d)))
                {
                    landing_lift_angle = -20d / 180d * Math.PI;
                    throttle = Math.Clamp(State.Vessel.Mass * State.Vessel.Gravity * 0.8d / State.Vessel.AvailableThrust, 0d, 1d);
                }
                else if (Trajectory.LiftEstimationForceMin > 1d)
                {
                    landing_lift_angle = 20d / 180d * Math.PI;
                }
                else
                {
                    Command.SetTargetDirection(Trajectory.EnterAtmosphereDirection);
                    //Console.WriteLine("{0:0.0}", distance);
                    landing_adjust_throttle = throttle;
                    return false;
                }
            }
            else if (Trajectory.NextBurnTime < 0.5d)
            {
                double f = Trajectory.LiftEstimationThrustAve * Math.Sin(Trajectory.LiftEstimationAngle);
                double diff = f - Trajectory.LiftEstimationForceAve;
                if (landing_lift_angle < 0d)
                {
                    if (diff < State.Vessel.Mass)
                        landing_lift_angle = 0d;
                    else
                        landing_lift_angle = -Trajectory.LiftEstimationAngle;
                }
                else if (landing_lift_angle > 0d)
                {
                    if (diff > -State.Vessel.Mass)
                        landing_lift_angle = 0d;
                    else
                        landing_lift_angle = Trajectory.LiftEstimationAngle;
                }
                else
                {
                    if (diff > State.Vessel.Mass * 1.5d)
                        landing_lift_angle = -Trajectory.LiftEstimationAngle;
                    else if (diff < -State.Vessel.Mass * 1.5d)
                        landing_lift_angle = Trajectory.LiftEstimationAngle;
                    else
                        landing_lift_angle = 0d;
                }
            }
            else
            {
                landing_lift_angle = 0d;
            }

            Vector3d tar_dir;
            if (throttle < 0d)
            {
                double turn_angle_ratio = Math.Clamp(distance * 50d / Math.Clamp(State.Vessel.Altitude - tar_altitude, 1000d, 10000d), 0d, 1d);
                double turn_angle_limit = Math.Max(0d, (State.Vessel.Altitude - tar_altitude) / 100d);
                double turn_angle = Math.Clamp(-landing_lift_angle * turn_angle_ratio, -turn_angle_limit, turn_angle_limit);
                Vector3d turn_v1 = Vector3d.Cross(Trajectory.ImpactPositionWithAction.Norm(), tar_pos.Norm()).Norm();
                Vector3d turn_v = turn_angle * turn_v1;
                Matrix3d r = turn_v.RotationMatrix();
                tar_dir = r * (-State.Vessel.Velocity).Norm();
                landing_adjust_throttle = -1d;
            }
            else
            {
                tar_dir = -tar_pos_v.Norm() * Math.Sin(landing_lift_angle) + State.Vessel.BodyUp * Math.Cos(landing_lift_angle);
                double dir_error = State.Vessel.Direction * tar_dir;
                landing_adjust_throttle = throttle < 0d ? -1d : throttle * (dir_error - 0.95d) * 20d;
            }

            Command.SetTargetDirection(tar_dir);

            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_pos_v.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame);
            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_dir.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame,
                30f);
            Conn.Drawing().AddLine(
                State.Vessel.Position.ToTuple(),
                Trajectory.ImpactPositionWithAction.ToTuple(),
                OrbitBody.ReferenceFrame);
            Console.WriteLine("{0:0.0}\t{1:0}\t{2:0}\t{3:0.00}\t{4:0.00}\t{5:0.000}",
                Trajectory.NextBurnTime,
                Trajectory.LiftEstimationThrustAve,
                Trajectory.LiftEstimationForceAve,
                landing_lift_angle / Math.PI * 180d,
                distance,
                landing_adjust_throttle);

            return false;
        }

        public bool LandingDirectionLast()
        {
            Command.SetTargetDirection(State.Vessel.BodyUp);
            return false;
        }

    }
}

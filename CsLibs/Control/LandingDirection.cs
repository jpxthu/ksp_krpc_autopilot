using KRPC.Client.Services.Drawing;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public bool LandingDirection(Vector3d tar_pos, double tar_altitude)
        {
            if (!Trajectory.ResultStable)
            {
                Command.SetTargetDirection(-State.Vessel.Velocity);
                return false;
            }

            Vector3d tar_pos_v_hori = tar_pos - (Trajectory.ImpactPositionWithAction + Trajectory.ImpactPositionWithActionChangeRate * 2d);
            tar_pos_v_hori = VectorHorizonPart(tar_pos_v_hori);
            double distance = tar_pos_v_hori.Length();
            double throttle = -1d;

            double up_ratio = Math.Max(State.Vessel.SurfUp * State.Vessel.Direction, 0.01d);
            double vel_tilt = Math.Acos(up_ratio);
            double grav_tilt = Math.Acos(State.Vessel.Gravity * 0.8d * State.Vessel.Mass / State.Vessel.AvailableThrust);
            double adjust_tilt = Math.Min(Math.PI / 2d, Math.Max(vel_tilt, grav_tilt));
            double adjust_throttle = 1d;

            if (Trajectory.NextBurnTime > 5d || (landing_adjust_throttle > 0.1d && Trajectory.NextBurnTime > 1d))
            {
                double v = State.Vessel.VelocityHorizonMag;
                double a = State.Vessel.AvailableThrust * adjust_throttle / State.Vessel.Mass * Math.Sin(adjust_tilt);
                double s = VectorHorizonPart(State.Vessel.Position - tar_pos).Length();
                double r = State.Vessel.Position.Length();
                s *= (r + State.Vessel.VelocityUp * v / a) / r;
                Vector3d tar = State.Vessel.Position - tar_pos;
                double distance_without_comp = (tar_pos - Trajectory.ImpactPositionWithAction).Length();
                if ((tar.Norm() * State.Vessel.SurfUp > Math.Cos(20d / 180d * Math.PI) ||
                     s < v * v / 2d / a + v * 10d ||
                     a * (Trajectory.NextBurnTime - 10d) * 5d < v) &&
                    ((distance_without_comp > 200d && landing_adjust_throttle >= 0d) ||
                     distance_without_comp > 200d + 100d * Trajectory.NextBurnTime))
                {
                    distance = distance_without_comp;
                    landing_lift_angle = -adjust_tilt;
                    throttle = adjust_throttle;
                }
                else if (Trajectory.LiftEstimationForceMin > 1d)
                {
                    landing_lift_angle = LANDING_ADJUST2_MIN_TILT_RAD;
                }
                else
                {
                    Command.SetTargetDirection(Trajectory.EnterAtmosphereDirection);
                    landing_adjust_throttle = -1d;
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
            double turn_angle = 0d;
            if (throttle < 0d)
            {
                double turn_angle_ratio = Math.Clamp(distance * 100d / Math.Clamp(State.Vessel.Altitude - tar_altitude, 1000d, 50000d), 0d, 1d);
                double turn_angle_limit = Math.Clamp((State.Vessel.Altitude - tar_altitude - 100d) / 500d, 0d, 1d);
                turn_angle = -landing_lift_angle * turn_angle_ratio * turn_angle_limit;
                Vector3d turn_v1 = Vector3d.Cross(Trajectory.ImpactPositionWithAction.Norm(), tar_pos.Norm()).Norm();
                Vector3d turn_v = turn_angle * turn_v1;
                Matrix3d r = turn_v.RotationMatrix();
                tar_dir = r * (-State.Vessel.Velocity).Norm();
            }
            else
            {
                Vector3d hori_dir = State.Vessel.VelocityHorizonMag < distance ?
                    tar_pos_v_hori : -State.Vessel.VelocityHorizon;
                tar_dir = hori_dir.Norm() * Math.Sin(adjust_tilt) + State.Vessel.SurfUp * Math.Cos(adjust_tilt);
            }

            double up_dir_ratio = MathLib.InverseLerpWithClamp(50d, 0d, State.Vessel.VelocityMag);
            tar_dir = tar_dir * (1d - up_dir_ratio) + State.Vessel.SurfUp * up_dir_ratio;
            tar_dir = tar_dir.Norm();
            Command.SetTargetDirection(tar_dir);
            double dir_error = State.Vessel.Direction * tar_dir;
            landing_adjust_throttle = throttle < 0d ? -1d : Math.Max(0d, throttle * (dir_error - 0.95d) * 20d);

            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(State.Vessel.Direction.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame, 40f);
            Conn.Drawing().AddDirection(
                SpaceCenter.TransformDirection(tar_dir.ToTuple(), OrbitBody.ReferenceFrame, ActiveVessel.ReferenceFrame),
                ActiveVessel.ReferenceFrame,
                30f);
            Conn.Drawing().AddLine(
                State.Vessel.Position.ToTuple(),
                Trajectory.ImpactPositionWithAction.ToTuple(),
                OrbitBody.ReferenceFrame);
            Conn.Drawing().AddLine(
                State.Vessel.Position.ToTuple(),
                tar_pos.ToTuple(),
                OrbitBody.ReferenceFrame);
            Console.WriteLine("{0:0.0}\t{1:0}\t{2:0}\t{3:0.00}\t{4:0.00}\t{5:0.000}",
                Trajectory.NextBurnTime,
                Trajectory.LiftEstimationThrustAve,
                Trajectory.LiftEstimationForceAve,
                landing_lift_angle / Math.PI * 180d,
                distance,
                turn_angle);

            return false;
        }

        public bool LandingDirectionLast()
        {
            //Command.SetTargetDirection(-State.Vessel.Velocity);
            Command.SetTargetDirection(State.Vessel.SurfUp);
            return false;
        }
    }
}

using KRPC.Client.Services.Drawing;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        //private StreamWriter sw;

        public bool LandingInit(double tar_altitude)
        {
            //sw = new StreamWriter("r.tsv");

            gear_deployed = false;
            landing_adjust_throttle = -1d;
            landing_lift_angle = 0d;
            landing_rcs_tar_acc_i = Vector3d.Zero;
            RcsAltitudeControl = true;

            Trajectory.CalculateStart(tar_altitude: tar_altitude);
            return true;
        }

        /// <summary>
        /// 高空调整姿态和点火，调整下落轨迹
        /// </summary>
        /// <param name="tar_pos">Meters in Body.ReferenceFrame</param>
        /// <param name="tar_altitude">Meters</param>
        /// <param name="heading">Radian</param>
        /// <param name="could_burn"></param>
        /// <returns></returns>
        public LandingAdjustBurnStatus AdjustLandingPosition(
            Vector3d tar_pos, double tar_altitude, bool could_burn, double? heading = null)
        {
            if (heading.HasValue)
                Command.SetHeadingAngle(heading.Value);
            else
                Command.StableHeading();

            if (!Trajectory.ResultAvailable || !Trajectory.ResultWithoutActionAvailable)
            {
                Command.SetThrottle(0d);
                Command.SetTargetDirection(State.Vessel.SurfUp);
                return LandingAdjustBurnStatus.UNAVAILABEL;
            }

            Vector3d tar_v;
            if (!Trajectory.ResultStable)
            {
                tar_v = TargetPositionCompensate(tar_pos, tar_altitude) - Trajectory.ImpactPositionWithoutAction;
                tar_v = VectorHorizonPart(tar_v).Norm();
                if (tar_v * State.Vessel.Direction < -0.5d)
                    tar_v = State.Vessel.SurfUp;
                Command.SetTargetDirection(tar_v);
                Command.SetThrottle(0d);
                return LandingAdjustBurnStatus.UNAVAILABEL;
            }

            tar_v = TargetPositionCompensate(tar_pos, tar_altitude) - Trajectory.ImpactPositionWithAction;
            double distance = tar_v.Length();
            tar_v = VectorHorizonPart(tar_v).Norm();
            double dir_error = State.Vessel.Direction * tar_v;
            double tar_t = Math.Clamp(distance / 15000d, 0.2d, 0.6d);
            double tar_t_ratio = Command.Throttle > 0.1d ? 1d :
                Math.Min(Math.Max(0d, (dir_error - 0.95d) * 20d), Math.Max(0d, 1d - State.Vessel.AngularVelocity.Length() * 20d));
            tar_t *= tar_t_ratio;

            if (Command.Throttle > 0.1d &&
                VectorHorizonPart(State.Vessel.Direction).Norm() * tar_v < 0.985d)
            {
                Command.SetThrottle(0d);
                return LandingAdjustBurnStatus.FINISHED;
            }
            else
            {
                Command.SetTargetDirection(tar_v);
                if (tar_t_ratio < 0.01d)
                {
                    Command.SetThrottle(0d);
                    return LandingAdjustBurnStatus.UNAVAILABEL;
                }
                else if (could_burn)
                {
                    Command.SetThrottle(tar_t);
                    return LandingAdjustBurnStatus.EXECUTING;
                }
                else
                {
                    Command.SetThrottle(0d);
                    return LandingAdjustBurnStatus.WAITING;
                }
            }
        }

        /// <summary>
        /// 调整落地位置后，控制火箭着陆。
        /// </summary>
        /// <param name="tar_pos">目标坐标</param>
        /// <param name="tar_altitude">目标海拔</param>
        /// <param name="rcs_layout">RCS 布局</param>
        /// <param name="gear_deploy_time">着陆架部署需要时间</param>
        /// <param name="heading">Radian</param>
        /// <returns></returns>
        public bool Landing(
            Vector3d tar_pos, double tar_altitude, RcsLayout rcs_layout,
            double gear_deploy_time,
            double? heading = null)
        {
            //if (RcsAltitudeControl && Trajectory.ResultAvailable && Trajectory.NextBurnTime < 0.5d)
            //    RcsAltitudeControl = false;
            tar_pos = TargetPositionCompensate(tar_pos, tar_altitude);

            //Conn.Drawing().Clear();
            if (State.Vessel.Altitude - tar_altitude < 10000d && -State.Vessel.VelocityUp < 20d)
            {
                LandingDirection(tar_pos, tar_altitude);
                //LandingDirectionLast();
                LandingRcsLast(tar_pos, rcs_layout);
                if (!gear_deployed && Trajectory.ImpactTime <= gear_deploy_time)
                    gear_deployed = ActiveVessel.Control.Gear = true;
                if (-State.Vessel.VelocityUp < MIN_LANDING_VELOCITY / 2d)
                    return true;
            }
            else
            {
                LandingDirection(tar_pos, tar_altitude);
                LandingRcs(tar_pos, tar_altitude, rcs_layout);
            }
            LandingThrust(tar_altitude);
            if (heading.HasValue)
                Command.SetHeadingAngle(heading.Value);
            else
                Command.StableHeading();

            return false;
        }
    }
}

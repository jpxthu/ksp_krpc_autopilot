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
            landing_stage = LandingStage.ADJUST;
            RcsAltitudeControl = true;

            Trajectory.CalculateStart(tar_altitude: tar_altitude);
            Command.SetTargetDirection(-State.Vessel.Velocity);
            return true;
        }

        /// <summary>
        /// 高空调整姿态和点火，调整下落轨迹
        /// </summary>
        /// <returns></returns>
        public bool AdjustLandingPosition(Vector3d tar_pos, double tar_altitude)
        {
            if (!Trajectory.ResultAvailable)
                return false;
            var tar_v = TargetPositionCompensate(tar_pos, tar_altitude) - Trajectory.ImpactPositionWithAction;
            var distance = tar_v.Length();
            tar_v = VectorHorizonPart(tar_v).Norm();
            var dir_error = State.Vessel.Direction * tar_v;
            var tar_t = Math.Clamp(distance / 15000d, 0.5d, 1d) *
                Math.Max(0d, (dir_error - 0.95d) * 20d);
            //Conn.Drawing().Clear();
            //Conn.Drawing().AddDirection(
            //    SpaceCenter.TransformDirection(tar_v.ToTuple(), ActiveVessel.Orbit.Body.ReferenceFrame, ActiveVessel.ReferenceFrame),
            //    ActiveVessel.ReferenceFrame);
            if (distance < 1000d && tar_v * State.Vessel.Direction < 0.985d)
            {
                Command.SetThrottle(0d);
                return true;
            }
            else
            {
                //Console.WriteLine(distance);
                Command.SetTargetDirection(tar_v);
                Command.SetThrottle(tar_t);
                return false;
            }
        }

        /// <summary>
        /// 调整落地位置后，控制火箭着陆。
        /// </summary>
        /// <param name="tar_pos">目标坐标</param>
        /// <param name="tar_altitude">目标海拔</param>
        /// <param name="rcs_layout">RCS 布局</param>
        /// <param name="gear_deploy_time">着陆架部署需要时间</param>
        /// <returns></returns>
        public bool Landing(
            Vector3d tar_pos, double tar_altitude, RcsLayout rcs_layout,
            double gear_deploy_time)
        {
            //if (RcsAltitudeControl && Trajectory.ResultAvailable && Trajectory.NextBurnTime < 0.5d)
            //    RcsAltitudeControl = false;
            tar_pos = TargetPositionCompensate(tar_pos, tar_altitude);

            Conn.Drawing().Clear();
            if (State.Vessel.Altitude - tar_altitude < 10000d && -State.Vessel.VelocityUp < 10d)
            {
                LandingDirectionLast();
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

            return false;
        }
    }
}

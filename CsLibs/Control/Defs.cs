using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public enum Status
        {
            FINISHED,
            UNFINISHED,
            FAIL,
            SUCCESS
        }

        public enum ApproachTargetStage
        {
            SHIP,
            PORT,
            FINISHED
        }
        public enum ApproachStage
        {
            ENGINE,
            MIX,
            PCS,
            DOCKING
        }

        public enum LandingAdjustBurnStatus
        {
            UNAVAILABEL,
            WAITING,
            EXECUTING,
            FINISHED,
            ABANDON
        }

        public enum RcsLayout
        {
            TOP,            // 顶端，远离发动机
            TAIL,           // 微端，靠近发动机
            SYMMETRICAL     // 对称
        }

        /// <summary>
        /// 目标可以查询细节的距离，m
        /// </summary>
        private const double TARGET_HAS_DETAILS_DISTANCE_THRESHOLD = 2000d;
        private const double TARGET_HAS_DETAILS_DISTANCE_THRESHOLD_SQUAR = TARGET_HAS_DETAILS_DISTANCE_THRESHOLD * TARGET_HAS_DETAILS_DISTANCE_THRESHOLD;

        private const double LAUNCH_ASCENT_ANGLE_OF_ATTACK_MAX = 5d / 180d * Math.PI;
        private const double LAUNCH_ASCENT_TURN_TIME = 20d;
        private const double LAUNCH_ASCENT_GRAVITY_TURN_VELOCITY = 100d;

        private const double LANDING_MAX_THROTTLE = 1d;
        private const double LANDING_ADJUST2_MIN_TILT_RAD = 20d / 180d * Math.PI;
        private const double LANDING_ADJUST2_TURN_TIME = 5d;

        private bool gear_deployed_;
        private double landing_adjust_throttle_;
        private double landing_lift_angle_;
        private double landing_min_velocity_;
        private Vector3d landing_rcs_tar_acc_i_;
        private bool rcs_altitude_control_;
    }
}

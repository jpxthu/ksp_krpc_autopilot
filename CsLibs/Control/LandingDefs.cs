using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        public enum RcsLayout
        {
            TOP,            // 顶端，远离发动机
            TAIL,           // 微端，靠近发动机
            SYMMETRICAL     // 对称
        }

        public enum LandingAdjustBurnStatus
        {
            UNAVAILABEL,
            WAITING,
            EXECUTING,
            FINISHED,
            ABANDON
        }

        private const double LANDING_MAX_THROTTLE = 1d;
        private const double LANDING_ADJUST2_MIN_TILT_RAD = 20d / 180d * Math.PI;
        private const double LANDING_MIN_VELOCITY = 5d;
        private const double LANDING_ADJUST2_TURN_TIME = 5d;

        private bool gear_deployed;
        private double landing_adjust_throttle;
        private double landing_lift_angle;
        private Vector3d landing_rcs_tar_acc_i;
        private bool rcs_altitude_control;
    }
}

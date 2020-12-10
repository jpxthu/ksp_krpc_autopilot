using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public class Command
    {
        public enum Type
        {
            STABLE,
            VALUE
        }

        public void SetTargetDirection(double x, double y, double z)
        {
            DirectionVector = new Vector3d(x, y, z).Norm();
        }

        public void SetTargetDirection(Vector3d v)
        {
            DirectionVector = v.Norm();
        }

        public void ReleaseTargetDirection()
        {
            DirectionVector = null;
        }

        public void StableHeading()
        {
            HeadingMode = Type.STABLE;
        }

        /// <summary>
        /// </summary>
        /// <param name="angle">Radian</param>
        public void SetHeadingAngle(double angle)
        {
            HeadingAngle = angle;
            HeadingMode = Type.VALUE;
        }

        /// <summary>
        /// 设定目标 throttle
        /// </summary>
        /// <param name="throttle">[0, 1]</param>
        public void SetThrottle(double throttle)
        {
            Throttle = throttle;
        }

        /// <summary>
        /// </summary>
        /// <param name="throttle">[0, 1]</param>
        public void SetRcsForward(double throttle)
        {
            RcsForward = throttle;
        }

        /// <summary>
        /// </summary>
        /// <param name="throttle">[0, 1]</param>
        public void SetRcsRight(double throttle)
        {
            RcsRight = throttle;
        }

        /// <summary>
        /// </summary>
        /// <param name="throttle">[0, 1]</param>
        public void SetRcsUp(double throttle)
        {
            RcsUp = throttle;
        }

        public Command()
        {
            HeadingMode = Type.STABLE;
            RcsRight = RcsUp = RcsForward = Throttle = 0d;
        }

        public Vector3d DirectionVector { get; private set; }
        public Type HeadingMode { get; private set; }
        /// <summary>
        /// Radian
        /// </summary>
        public double HeadingAngle { get; private set; }
        public double RcsRight { get; private set; }
        public double RcsUp { get; private set; }
        public double RcsForward { get; private set; }
        public double Throttle { get; private set; }
    }
}

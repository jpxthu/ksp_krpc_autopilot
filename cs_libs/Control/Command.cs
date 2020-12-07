using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public class Command
    {
        public void SetTargetDirection(double x, double y, double z)
        {
            DirectionVector = new Vector3d(x, y, z).Norm();
        }

        public void SetTargetDirection(Vector3d v)
        {
            DirectionVector = v.Norm();
        }

        public void SetHeading(double heading)
        {
            Heading = heading;
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
            Heading = 0d;
            RcsRight = RcsUp = RcsForward = Throttle = 0d;
        }

        public Vector3d DirectionVector { get; private set; }
        public double Heading { get; private set; }
        public double RcsRight { get; private set; }
        public double RcsUp { get; private set; }
        public double RcsForward { get; private set; }
        public double Throttle { get; private set; }
    }
}

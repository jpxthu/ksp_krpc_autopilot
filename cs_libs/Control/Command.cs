using KRPC.Client.Services.SpaceCenter;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    class Command
    {
        private enum DirectionCommandType
        {
            PITCH,
            VECTOR
        }

        /// <summary>
        /// 设定目标 pitch 和 heading angles
        /// </summary>
        /// <param name="pitch">目标 pitch angle，degree，[-90°, +90°]</param>
        /// <param name="heading">目标 heading angle，degree，[0°, 360°]</param>
        public void SetTargetPitchAndHeading(double pitch, double heading)
        {
            DirectionType = DirectionCommandType.PITCH;
            Pitch = pitch;
            Heading = heading;
        }

        public void SetTargetDirection(double x, double y, double z)
        {
            DirectionType = DirectionCommandType.VECTOR;
            DirectionVector = new Vector3d(x, y, z).Norm();
        }

        public void SetTargetDirection(Vector3d v)
        {
            DirectionType = DirectionCommandType.VECTOR;
            DirectionVector = v.Norm();
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

        public void Engage()
        {
            if (!Engaged) {
                AutoPilot.Engage();
                Engaged = true;
            }
        }

        public void DisEngage()
        {
            if (Engaged) {
                AutoPilot.Disengage();
                Engaged = false;
            }
        }

        public void Execute()
        {
            switch (DirectionType) {
                case DirectionCommandType.PITCH:
                    AutoPilot.TargetPitchAndHeading(Convert.ToSingle(Pitch), Convert.ToSingle(Heading));
                    break;
                case DirectionCommandType.VECTOR:
                    AutoPilot.TargetDirection = new Tuple<double, double, double>(
                        DirectionVector.X,
                        DirectionVector.Y,
                        DirectionVector.Z);
                    break;
            }

            ActiveVessel.Control.Throttle = Convert.ToSingle(Throttle);
            ActiveVessel.Control.Forward = Convert.ToSingle(RcsForward);
            ActiveVessel.Control.Right = Convert.ToSingle(RcsRight);
            ActiveVessel.Control.Up = Convert.ToSingle(RcsUp);
        }

        public Command(Vessel vessel)
        {
            ActiveVessel = vessel;
            AutoPilot = vessel.AutoPilot;
            Engaged = false;
            DirectionType = DirectionCommandType.PITCH;
            Pitch = 0.0;
            Heading = 0.0;
            Throttle = 0.0;

            vessel.AutoPilot.ReferenceFrame = vessel.Orbit.Body.ReferenceFrame;
        }

        private Vessel ActiveVessel { get; }
        private AutoPilot AutoPilot { get; }
        private bool Engaged { get; set; }
        private DirectionCommandType DirectionType { get; set; }
        private Vector3d DirectionVector { get; set; }
        private double Pitch { get; set; }
        private double Heading { get; set; }
        private double Throttle { get; set; }
        private double RcsRight { get; set; }
        private double RcsUp { get; set; }
        private double RcsForward { get; set; }
    }
}

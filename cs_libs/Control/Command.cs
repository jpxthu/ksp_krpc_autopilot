using KRPC.Client.Services.SpaceCenter;
using KrpcLibs.Math;
using System;

namespace KrpcLibs
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
            DirectionVector = new Vector3d(x, y, z);
        }

        /// <summary>
        /// 设定目标 throttle
        /// </summary>
        /// <param name="throttle">[0, 1]</param>
        public void SetThrottle(double throttle)
        {
            Throttle = throttle;
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
                Console.WriteLine("Throttle: {0}\tPitch: {1}", Throttle, Pitch);
                break;
            case DirectionCommandType.VECTOR:
                AutoPilot.TargetDirection = new Tuple<double, double, double>(
                    DirectionVector.X,
                    DirectionVector.Y,
                    DirectionVector.Z);
                break;
            }

            ActiveVessel.Control.Throttle = Convert.ToSingle(Throttle);

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
        }

        private Vessel ActiveVessel { get; set; }
        private AutoPilot AutoPilot { get; }
        private bool Engaged { get; set; }
        private DirectionCommandType DirectionType { get; set; }
        private Vector3d DirectionVector { get; set; }
        private double Pitch { get; set; }
        private double Heading { get; set; }
        private double Throttle { get; set; }
    }
}

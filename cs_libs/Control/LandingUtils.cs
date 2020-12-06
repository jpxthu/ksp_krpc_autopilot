using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool RcsAltitudeControl
        {
            get => rcs_altitude_control;
            set
            {
                foreach (var rcs in ActiveVessel.Parts.RCS)
                {
                    rcs.RollEnabled = value;
                    rcs.PitchEnabled = value;
                    rcs.YawEnabled = value;
                }
                rcs_altitude_control = value;
            }
        }

        private double EngineMaxHorizonForce()
        {
            double engine_gimbal_thrust_limit = 0d;
            foreach (var engine in ActiveVessel.Parts.Engines)
            {
                if (engine.Active)
                {
                    engine_gimbal_thrust_limit += Convert.ToDouble(engine.Thrust) *
                        Math.Sin(Convert.ToDouble(engine.GimbalRange) / 180d * Math.PI);
                }
            }
            return engine_gimbal_thrust_limit;
        }

        private double RcsMaxHorizonForce(RcsLayout rcs_layout)
        {
            double rcs_force_limit = Math.Min(
                Math.Min(State.Vessel.AvailableRCSForce.Item1.X, State.Vessel.AvailableRCSForce.Item1.Z),
                Math.Min(-State.Vessel.AvailableRCSForce.Item2.X, -State.Vessel.AvailableRCSForce.Item2.Z));
            if (rcs_layout == RcsLayout.TOP)
                rcs_force_limit = Math.Min(rcs_force_limit, EngineMaxHorizonForce());
            return rcs_force_limit;
        }

        private void RcsSetByForce(double force_right, double force_up)
        {
            Command.SetRcsRight(force_right >= 0d ?
                force_right / State.Vessel.AvailableRCSForce.Item1.X :
                -force_right / State.Vessel.AvailableRCSForce.Item2.X);
            Command.SetRcsUp(force_up >= 0d ?
                -force_up / State.Vessel.AvailableRCSForce.Item2.Z :
                force_up / State.Vessel.AvailableRCSForce.Item1.Z);
        }

        private Vector3d VectorHorizonPart(Vector3d v)
        {
            return v - v * State.Vessel.BodyUp * State.Vessel.BodyUp;
        }

        private Vector3d TargetPositionCompensate(Vector3d tar_pos, double tar_altitude)
        {
            Vector3d dir = VectorHorizonPart(State.Vessel.Velocity.Norm());
            double ratio = Math.Min(100d, (State.Vessel.Altitude - tar_altitude) / 100d);
            return tar_pos + dir * ratio;
        }
    }
}

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
            try
            {
                foreach (var engine in ActiveVessel.Parts.Engines)
                {
                    if (engine.Active)
                    {
                        engine_gimbal_thrust_limit += Convert.ToDouble(engine.Thrust) *
                            Math.Sin(Convert.ToDouble(engine.GimbalRange) / 180d * Math.PI);
                    }
                }
            }
            catch (Exception) { }
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
            double limit = Math.Clamp((1d - Math.Max(Math.Abs(Command.Pitch), Math.Abs(Command.Yaw))) * 2d, 0d, 1d);
            double right = force_right >= 0d ?
                force_right / State.Vessel.AvailableRCSForce.Item1.X :
                -force_right / State.Vessel.AvailableRCSForce.Item2.X;
            double up = force_up >= 0d ?
                -force_up / State.Vessel.AvailableRCSForce.Item2.Z :
                force_up / State.Vessel.AvailableRCSForce.Item1.Z;
            Command.SetRcsRight(Math.Clamp(right, -limit, limit));
            Command.SetRcsUp(Math.Clamp(up, -limit, limit));
        }

        private Vector3d VectorHorizonPart(Vector3d v)
        {
            return v - v * State.Vessel.SurfUp * State.Vessel.SurfUp;
        }

        private Vector3d TargetPositionCompensate(Vector3d tar_pos, double tar_altitude)
        {
            //Vector3d dir = State.Vessel.VelocityHorizon.Norm();
            Vector3d dir = VectorHorizonPart(tar_pos - State.Vessel.Position);
            double ratio = 0.3d;// Math.Min(200d, (State.Vessel.Altitude - tar_altitude) / 100d);
            return tar_pos + dir * ratio;
        }

        private Vector3d TargetPositionCompensate2(Vector3d tar_pos, double tar_altitude)
        {
            //Vector3d dir = State.Vessel.VelocityHorizon;
            //dir = dir.Norm() * Math.Min(2d, dir.Length());
            Vector3d dir = VectorHorizonPart(tar_pos - State.Vessel.Position).Norm();
            double ratio = Math.Clamp(Math.Sqrt(Math.Max(0d, State.Vessel.Altitude - tar_altitude - 2000d)), 0d, 100d);
            return tar_pos + dir * ratio;
        }
    }
}

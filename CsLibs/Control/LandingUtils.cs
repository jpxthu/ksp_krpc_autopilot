using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private bool RcsAltitudeControl
        {
            get => rcs_altitude_control_;
            set
            {
                foreach (var rcs in ActiveVessel.Parts.RCS)
                {
                    rcs.RollEnabled = value;
                    rcs.PitchEnabled = value;
                    rcs.YawEnabled = value;
                }
                rcs_altitude_control_ = value;
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

        private double RcsMaxHorizonForce(RcsLayout rcs_layout = RcsLayout.SYMMETRICAL)
        {
            double rcs_force_limit = Math.Min(
                Math.Min(State.Vessel.AvailableRCSForce.Item1.X, State.Vessel.AvailableRCSForce.Item1.Z),
                Math.Min(-State.Vessel.AvailableRCSForce.Item2.X, -State.Vessel.AvailableRCSForce.Item2.Z));
            if (rcs_layout == RcsLayout.TOP)
                rcs_force_limit = Math.Min(rcs_force_limit, EngineMaxHorizonForce());
            return rcs_force_limit;
        }

        private void RcsSetByForce(double force_right, double force_up, double force_forward = 0d)
        {
            double limit = Math.Clamp((1d - Math.Max(Math.Abs(Command.Pitch), Math.Abs(Command.Yaw))) * 2d, 0d, 1d);
            double right = force_right >= 0d ?
                force_right / State.Vessel.AvailableRCSForce.Item1.X :
                -force_right / State.Vessel.AvailableRCSForce.Item2.X;
            double forward = force_forward >= 0d ?
                force_forward / State.Vessel.AvailableRCSForce.Item1.Y :
                -force_forward / State.Vessel.AvailableRCSForce.Item2.Y;
            double up = force_up >= 0d ?
                -force_up / State.Vessel.AvailableRCSForce.Item2.Z :
                force_up / State.Vessel.AvailableRCSForce.Item1.Z;
            Command.SetRcsRight(Math.Clamp(right, -limit, limit));
            Command.SetRcsForward(Math.Clamp(forward, -1d, 1d));
            Command.SetRcsUp(Math.Clamp(up, -limit, limit));
        }

        private void RcsSetByForce(Vector3d force)
        {
            RcsSetByForce(
                force * State.Vessel.Right,
                force * State.Vessel.Up,
                force * State.Vessel.Forward);
        }

        private void RcsForceOnSpecificDirection(
            Vector3d dir,
            out double force_on_dir, out double force_on_dir_reverse)
        {
            var forces = State.Vessel.AvailableRCSForce;

            double right_ratio = State.Vessel.Right * dir;
            double right = forces.Item1.X * right_ratio;
            double left = forces.Item2.X * right_ratio;
            double forward_ratio = State.Vessel.Forward * dir;
            double forward = forces.Item1.Y * forward_ratio;
            double backward = forces.Item2.Y * forward_ratio;
            double bottom_ratio = -State.Vessel.Up * dir;
            double bottom = forces.Item1.Z * bottom_ratio;
            double up = forces.Item2.Z * bottom_ratio;

            force_on_dir =
                Math.Max(0d, right) +
                Math.Max(0d, left) +
                Math.Max(0d, forward) +
                Math.Max(0d, backward) +
                Math.Max(0d, bottom) +
                Math.Max(0d, up);
            force_on_dir_reverse =
                Math.Max(0d, -right) +
                Math.Max(0d, -left) +
                Math.Max(0d, -forward) +
                Math.Max(0d, -backward) +
                Math.Max(0d, -bottom) +
                Math.Max(0d, -up);
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
            Vector3d dir = VectorHorizonPart(tar_pos - State.Vessel.Position);
            double ratio = 0d;// Math.Clamp(Math.Sqrt(Math.Max(0d, State.Vessel.Altitude - tar_altitude - 2000d)), 0d, 200d);
            //ratio = Math.Min(ratio, dir.Length());
            return tar_pos + dir.Norm() * ratio;
        }
    }
}

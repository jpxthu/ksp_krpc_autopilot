using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private AltitudeController altitude_controller_;

        private void AltitudeControllerInit(
            double linear_k = 1d,
            double max_act = 0.8d,
            double kp = 2d,
            double ki = 0d)
        {
            altitude_controller_ = new AltitudeController(linear_k, max_act, kp, ki);
        }

        private void AltitudeControl()
        {
            if (Command.DirectionVector is null)
                return;

            Vector3d pitch_axis = State.Vessel.Right;
            Vector3d yaw_axis = -State.Vessel.Up;
            Vector3d torque = State.Vessel.AvailableTorque.Item1;
            Vector3d moi = State.Vessel.MomentOfInertia;
            altitude_controller_.LinearK = MathLib.Lerp(2d, 1d, MathLib.InverseLerpWithClamp(0d, 0.1d, Command.Throttle));
            altitude_controller_.Kp = altitude_controller_.LinearK * 2d;
            altitude_controller_.MaxPitchAngAcc = -torque.X / moi.X * pitch_axis;
            altitude_controller_.MaxYawAngAcc = -torque.Z / moi.Z * yaw_axis;
            altitude_controller_.Update(
                Command.DirectionVector,
                State.Vessel.Direction,
                State.Vessel.AngularVelocity,
                out double pitch,
                out double yaw);

            Vector3d turn_v = Vector3d.Cross(State.Vessel.SurfUp, State.Vessel.Direction);
            double heading = Command.Heading;
            Vector3d origin_down = State.Vessel.SurfNorth * Math.Sin(heading) + State.Vessel.SurfEast * Math.Cos(heading);
            Vector3d tar_down = turn_v.RotationMatrix() * origin_down;
            Vector3d roll_v = Vector3d.Cross(-State.Vessel.Up, tar_down);
            double roll_ang = roll_v * State.Vessel.Forward;
            double roll_torque = torque.Y / moi.Y;
            LinearPlanner.Hover(roll_ang, 1d, roll_torque, roll_torque, out double vel, out double acc);
            double vel_err = State.Vessel.AngularVelocity * State.Vessel.Forward - vel;
            double roll = Math.Clamp((acc + vel_err * 2d) / roll_torque, -1d, 1d);

            double max_py = 1d - Math.Abs(roll);
            pitch = Math.Clamp(pitch, -max_py, max_py);
            yaw = Math.Clamp(yaw, -max_py, max_py);

            ActiveVessel.Control.Pitch = Convert.ToSingle(pitch);
            ActiveVessel.Control.Roll = Convert.ToSingle(roll);
            ActiveVessel.Control.Yaw = Convert.ToSingle(yaw);
        }
    }
}

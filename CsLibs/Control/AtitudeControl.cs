using KRPC.Client.Services.Drawing;
using KrpcAutoPilot.Utils;
using System;

namespace KrpcAutoPilot
{
    public partial class Control
    {
        private AtitudeController atitude_controller_;
        private double altitude_controller_roll_linear_k_;
        private double altitude_controller_roll_max_act_;
        private double altitude_controller_roll_kp_;
        private double altitude_controller_roll_int_;
        private double altitude_controller_roll_ki_;

        private void AltitudeControllerInit(
            double linear_k = 1.5d,
            double max_act = 0.6d,
            double kp = 1.5d,
            double ki = 0.1d)
        {
            atitude_controller_ = new AtitudeController(linear_k, max_act, kp, ki);
            altitude_controller_roll_linear_k_ = 0.8d;
            altitude_controller_roll_max_act_ = max_act;
            altitude_controller_roll_kp_ = kp * 2d;
            altitude_controller_roll_int_ = 0d;
            altitude_controller_roll_ki_ = ki * 2d;
        }

        private bool AtitudeControlSetRPY(double roll, double pitch, double yaw)
        {
            Command.Roll = roll;
            Command.Pitch = pitch;
            Command.Yaw = yaw;
            try
            {
                ActiveVessel.Control.Pitch = Convert.ToSingle(pitch);
                ActiveVessel.Control.Roll = Convert.ToSingle(roll);
                ActiveVessel.Control.Yaw = Convert.ToSingle(yaw);
            }
            catch (Exception e)
            {
                Console.WriteLine(
                    "Error when controlling attitude. Vessel <{0}> may out of distance or crashed. " +
                    "Error message: {1}", VesselName, e.Message);
                return false;
            }
            return true;
        }

        private bool AtitudeControl()
        {
            if (Command.DirectionVector is null)
                return AtitudeControlSetRPY(0d, 0d, 0d);

            Vector3d pitch_axis = State.Vessel.Right;
            Vector3d yaw_axis = -State.Vessel.Up;
            Vector3d torque = State.Vessel.AvailablePosTorque;
            Vector3d moi = State.Vessel.MomentOfInertia;
            //Console.WriteLine("{0}\t{1}", torque.ToString("0.000"), State.Vessel.AvailableTorque.Item2.ToString("0.000"));
            //atitude_controller_.LinearK = 1d;// MathLib.Lerp(2d, 1d, MathLib.InverseLerpWithClamp(0d, 0.1d, Command.Throttle));
            //atitude_controller_.Kp = atitude_controller_.LinearK * 2d;
            atitude_controller_.MaxPitchAngAcc = -torque.X / moi.X * pitch_axis;
            atitude_controller_.MaxYawAngAcc = -torque.Z / moi.Z * yaw_axis;
            atitude_controller_.Update(
                Command.DirectionVector,
                State.Vessel.Direction,
                State.Vessel.AngularVelocity,
                out double pitch,
                out double yaw);

            double acc = 0d, vel = 0d;
            double roll_torque = torque.Y / moi.Y;
            if (Command.HeadingMode == Command.Type.VALUE)
            {
                Vector3d turn_v = Vector3d.Cross(State.Vessel.SurfUp, State.Vessel.Direction);
                double heading = Command.HeadingAngle;
                Vector3d origin_down = State.Vessel.SurfNorth * Math.Sin(heading) + State.Vessel.SurfEast * Math.Cos(heading);
                Vector3d tar_down = turn_v.RotationMatrix() * origin_down;
                Vector3d roll_v = Vector3d.Cross(-State.Vessel.Up, tar_down);
                double roll_ang = roll_v * State.Vessel.Forward;
                LinearPlanner.Hover(roll_ang,
                    altitude_controller_roll_linear_k_,
                    roll_torque * altitude_controller_roll_max_act_,
                    roll_torque * altitude_controller_roll_max_act_,
                    out vel, out acc);
            }
            double vel_err = State.Vessel.AngularVelocity * State.Vessel.Forward - vel;
            double roll = (acc + vel_err * altitude_controller_roll_kp_) / roll_torque;
            altitude_controller_roll_int_ = Math.Clamp(altitude_controller_roll_int_ + roll * altitude_controller_roll_ki_, -0.5d, 0.5d);
            roll = Math.Clamp(roll + altitude_controller_roll_int_, -1d, 1d);

            double max_py = Math.Max(0.2d, 1d - Math.Abs(roll) / 1.5d);
            pitch = Math.Clamp(pitch, -max_py, max_py);
            yaw = Math.Clamp(yaw, -max_py, max_py);
            //return AtitudeControlSetRPY(0, 0, 0);
            return AtitudeControlSetRPY(roll, pitch, yaw);
        }
    }
}

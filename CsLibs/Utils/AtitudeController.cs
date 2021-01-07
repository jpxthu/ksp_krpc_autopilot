using System;

namespace KrpcAutoPilot.Utils
{
    public class AtitudeController
    {
        public Vector3d MaxPitchAngAcc { get; set; }
        public Vector3d MaxYawAngAcc { get; set; }
        public double LinearK { get; set; }
        // (0, 1]
        public double MaxAct { get; set; }
        public double Kp { get; set; }
        public double Ki { get; set; }
        private double int_pitch_;
        private double int_yaw_;

        /// <summary>
        /// </summary>
        /// <param name="max_act">(0, 1]</param>
        public AtitudeController(double linear_k, double max_act, double kp, double ki = 0d)
        {
            Reset(linear_k, max_act, kp, ki);
        }

        public bool Update(
            Vector3d tar_dir, Vector3d cur_dir, Vector3d cur_vel,
            out double pitch, out double yaw)
        {
            if (MaxPitchAngAcc == Vector3d.Zero ||
                MaxYawAngAcc == Vector3d.Zero)
            {
                pitch = yaw = 0d;
                return false;
            }

            Vector3d tar_ang_v = Vector3d.Cross(tar_dir, cur_dir);
            double tar_ang = tar_ang_v.Length();
            Vector3d tar_vel_v, tar_acc_v;
            if (tar_ang == 0d)
            {
                tar_vel_v = Vector3d.Zero;
                tar_acc_v = Vector3d.Zero;
            }
            else
            {
                Vector3d tar_ang_v_norm = tar_ang_v.Norm();
                double max_acc = (Math.Abs(tar_ang_v_norm * MaxPitchAngAcc) + Math.Abs(tar_ang_v_norm * MaxYawAngAcc)) * MaxAct;

                LinearPlanner.Hover(tar_ang, LinearK, max_acc, max_acc, out double tar_vel, out double tar_acc);
                LinearPlanner.Hover(2d * Math.PI - tar_ang, LinearK, max_acc, max_acc, out double tar_vel2, out double tar_acc2);
                double vel = cur_vel * tar_ang_v_norm;
                if (Math.Abs(vel - tar_vel) < Math.Abs(vel - tar_vel2))
                {
                    tar_vel_v = tar_vel * tar_ang_v_norm;
                    tar_acc_v = tar_acc * tar_ang_v_norm;
                }
                else
                {
                    tar_vel_v = -tar_vel2 * tar_ang_v_norm;
                    tar_acc_v = -tar_acc2 * tar_ang_v_norm;
                }
            }

            tar_acc_v += (tar_vel_v - cur_vel) * Kp;
            pitch = tar_acc_v * MaxPitchAngAcc.Norm() / MaxPitchAngAcc.Length();
            int_pitch_ = Math.Clamp(int_pitch_ + pitch * Ki, -1d, 1d);
            pitch = Math.Clamp(pitch + int_pitch_, -1d, 1d);
            yaw = tar_acc_v * MaxYawAngAcc.Norm() / MaxYawAngAcc.Length();
            int_yaw_ = Math.Clamp(int_yaw_ + yaw * Ki, -1d, 1d);
            yaw = Math.Clamp(yaw + int_yaw_, -1d, 1d);
            return true;
        }

        public void Reset(double linear_k, double max_act, double kp, double ki)
        {
            LinearK = linear_k;
            MaxAct = max_act;
            Kp = kp;
            Ki = ki;
            int_pitch_ = int_yaw_ = 0d;
        }
    }
}

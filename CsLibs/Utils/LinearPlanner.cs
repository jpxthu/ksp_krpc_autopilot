using System;

namespace KrpcAutoPilot.Utils
{
    public class LinearPlanner
    {
        public static void Hover(
            double s,
            double k, double max_acc_pos, double max_acc_neg,
            out double vel, out double acc)
        {
            double k2 = k * k;
            double sx_pos = max_acc_pos / k2;
            double sx_neg = max_acc_neg / k2;
            if (s > sx_pos)
            {
                double sb = max_acc_pos / 2d / k2;
                vel = -Math.Sqrt(2d * max_acc_pos * (s - sb));
                acc = max_acc_pos;
            }
            else if (s < -sx_neg)
            {
                double sb = max_acc_neg / 2d / k2;
                vel = Math.Sqrt(-2d * max_acc_neg * (s + sb));
                acc = -max_acc_neg;
            }
            else
            {
                vel = -k * s;
                acc = k2 * s;
            }
        }

        public static void OneWay(
            double s,
            double k, double max_acc, double min_vel,
            out double vel, out double acc)
        {
            double k2 = k * k;
            double sx = max_acc / k2;
            double s_min = min_vel / k;
            s += s_min;
            if (s < s_min)
            {
                vel = -min_vel;
                acc = 0d;
            }
            else if (s > sx)
            {
                double sb = max_acc / 2d / k2;
                vel = -Math.Sqrt(2d * max_acc * (s - sb));
                acc = max_acc;
            }
            else
            {
                vel = -k * s;
                acc = k2 * s;
            }
        }
    }
}

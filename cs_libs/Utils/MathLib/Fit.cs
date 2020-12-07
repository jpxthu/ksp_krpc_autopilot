using System.Linq;

namespace KrpcAutoPilot.Utils
{
    partial class MathLib
    {
        /// <summary>
        /// y = ax + b
        /// </summary>
        public static bool LinearFit(double[] x, double[] y, out double a, out double b)
        {
            int n = x.Length;
            double sum_x2 = x.Select(o => o * o).Sum();
            double sum_x = x.Sum();
            double sum_y = y.Sum();
            double[] xy = new double[n];
            for (int i = 0; i < n; i++)
                xy[i] = x[i] * y[i];
            double sum_xy = xy.Sum();
            double c = n * sum_x2 - sum_x * sum_x;
            if (c == 0d)
            {
                a = b = 0d;
                return false;
            }
            a = (n * sum_xy - sum_x * sum_y) / c;
            b = (sum_x2 * sum_y - sum_x * sum_xy) / c;
            return true;
        }

        /// <summary>
        /// y = ax + b
        /// </summary>
        public static bool LinearFit(double[] x, double[] y, double test_x, out double test_y)
        {
            if (LinearFit(x, y, out double a, out double b))
            {
                test_y = a * test_x + b;
                return true;
            }
            else
            {
                test_y = 0d;
                return false;
            }
        }
    }
}

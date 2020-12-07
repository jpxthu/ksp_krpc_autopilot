using System;

namespace KrpcAutoPilot.Utils
{
    partial class MathLib
    {
        public static double Lerp(double x1, double x2, double ratio)
        {
            return x1 + ratio * (x2 - x1);
        }
        public static double InverseLerp(double x1, double x2, double x)
        {
            return (x - x1) / (x2 - x1);
        }
        public static double InverseLerpWithClamp(double x1, double x2, double x)
        {
            return Math.Clamp((x - x1) / (x2 - x1), 0d, 1d);
        }
        public static Vector3d Lerp(Vector3d x1, Vector3d x2, double ratio)
        {
            return x1 + ratio * (x2 - x1);
        }
    }
}

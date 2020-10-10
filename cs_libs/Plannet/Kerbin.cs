using System;

namespace KrpcLibs.Plannet
{
    class Kerbin
    {
        public double AirDensity(double altitude)
        {
            return 3.407 * System.Math.Exp(-System.Math.Pow((altitude + 18250) / 17990, 2));
        }
    }
}

namespace KrpcAutoPilot.Utils
{
    static class Common
    {
        /// <summary>
        /// 空气密度
        /// </summary>
        /// <param name="pressure">静压，Pa</param>
        /// <param name="temperature">温度，K</param>
        /// <returns>空气密度，kg/m^3</returns>
        public static double AirDensity(double pressure, double temperature)
        {
            return pressure / Constants.Common.SPECIFIC_GAS_CONSTANT / temperature;
        }

        public static double AirDensityKerbin(double altitude)
        {
            double tmp = (altitude + 18250) / 17990;
            return 3.407 * System.Math.Exp(-tmp * tmp);
        }

        /// <summary>
        /// 动压
        /// </summary>
        /// <param name="pressure">静压，Pa</param>
        /// <param name="temperature">温度，K</param>
        /// <param name="velocity">速度，m/s</param>
        /// <returns>动压，Pa</returns>
        public static double DynamicPressure(double pressure, double temperature, double velocity)
        {
            return 0.5 * AirDensity(pressure, temperature) * velocity * velocity;
        }

        /// <summary>
        /// 动压
        /// </summary>
        /// <param name="air_density">空气密度，kg/m^3</param>
        /// <param name="velocity">速度，m/s</param>
        /// <returns>动压，Pa</returns>
        public static double DynamicPressure(double air_density, double velocity)
        {
            return 0.5 * air_density * velocity * velocity;
        }
    }
}

#include "Constants.hpp"

namespace kRPC {
namespace Libs {

/// <summary>
/// 空气密度
/// </summary>
/// <param name="pressure">静压，Pa</param>
/// <param name="temperature">温度，K</param>
/// <returns>空气密度，kg/m^3</returns>
double AirDensity(double pressure, double temperature) {
    return pressure / Constants::SPECIFIC_GAS_CONSTANT / temperature;
}

/// <summary>
/// 动压
/// </summary>
/// <param name="pressure">静压，Pa</param>
/// <param name="temperature">温度，K</param>
/// <param name="velocity">速度，m/s</param>
/// <returns>动压，Pa</returns>
double DynamicPressure(double pressure, double temperature, double velocity) {
    return 0.5 * AirDensity(pressure, temperature) * velocity * velocity;
}

}
}
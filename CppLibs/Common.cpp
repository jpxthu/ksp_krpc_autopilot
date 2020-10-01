#include "Constants.hpp"

namespace kRPC {
namespace Libs {

/// <summary>
/// �����ܶ�
/// </summary>
/// <param name="pressure">��ѹ��Pa</param>
/// <param name="temperature">�¶ȣ�K</param>
/// <returns>�����ܶȣ�kg/m^3</returns>
double AirDensity(double pressure, double temperature) {
    return pressure / Constants::SPECIFIC_GAS_CONSTANT / temperature;
}

/// <summary>
/// ��ѹ
/// </summary>
/// <param name="pressure">��ѹ��Pa</param>
/// <param name="temperature">�¶ȣ�K</param>
/// <param name="velocity">�ٶȣ�m/s</param>
/// <returns>��ѹ��Pa</returns>
double DynamicPressure(double pressure, double temperature, double velocity) {
    return 0.5 * AirDensity(pressure, temperature) * velocity * velocity;
}

}
}
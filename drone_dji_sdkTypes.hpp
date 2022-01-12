#ifndef drone_dji_sdk_TYPES_HPP
#define drone_dji_sdk_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Eigen.hpp>
#include <base/Angle.hpp>
namespace drone_dji_sdk
{
    enum BUTTON_ACTION
    {
        TAKEOFF_ACTIVATE,
        LANDING_ACTIVATE,
        CONTROL_ACTIVATE,
        MISSION_ACTIVATE
    };
    struct VehicleSetpoint
    {
        base::Vector3d setpoint;
        base::Angle heading;
    };
}

#endif

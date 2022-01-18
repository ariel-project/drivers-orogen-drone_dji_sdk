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
#include <base/Time.hpp>
namespace drone_dji_sdk
{
    enum COMMAND_ACTION
    {
        TAKEOFF_ACTIVATE,
        PRE_LANDING_ACTIVATE,
        LANDING_ACTIVATE,
        GO_TO_ACTIVATE,
        MISSION_ACTIVATE
    };
    struct VehicleSetpoint
    {
        base::Time time;
        base::Vector3d position;
        base::Angle heading;
    };
}

#endif

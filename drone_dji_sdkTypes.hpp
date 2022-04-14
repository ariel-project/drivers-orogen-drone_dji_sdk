#ifndef drone_dji_sdk_TYPES_HPP
#define drone_dji_sdk_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Angle.hpp>
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include "drone_control/Command.hpp"

namespace drone_dji_sdk
{
    enum AuthorityRequestResult
    {
        Success,
        Failure
    };

    enum ControlDevice
    {
        RemoteController,
        MobileApp,
        SDK
    };

    enum DeviceFlightStatus
    {
        Close,
        Opened
    };

    struct Status
    {
        AuthorityRequestResult authority_status;
        ControlDevice control_device;
        DeviceFlightStatus device_flight_status;
        drone_control::FlightStatus flight_status;
    };
} // namespace drone_dji_sdk

#endif

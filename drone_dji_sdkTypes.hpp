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

    struct Waypoint
    {
        uint8_t index;       /*!< Index to be uploaded */
        float latitude;  /*!< Latitude (radian) */
        float longitude; /*!< Longitude (radian) */
        float altitude;  /*!< Altitude (relative altitude from takeoff point) */
        float damping;   /*!< Bend length (effective coordinated turn mode only) */
        int16_t yaw;         /*!< Yaw (degree) */
        int16_t gimbalPitch; /*!< Gimbal pitch */
        uint8_t turnMode;    /*!< Turn mode <br> */
        /*!< 0: clockwise <br>*/
        /*!< 1: counter-clockwise <br>*/
        uint8_t reserved[8]; /*!< Reserved */
        uint8_t hasAction;   /*!< Action flag <br>*/
        /*!< 0: no action <br>*/
        /*!< 1: has action <br>*/
        uint16_t actionTimeLimit; /*!< Action time limit */
        uint8_t actionNumber; /*!< Total number of actions */
        uint8_t actionRepeat; /*!< Total running times */
        uint8_t commandList[16];  /*!< Command list */
        uint16_t commandParameter[16];
    };
}

#endif

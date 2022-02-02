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
    enum CommandAction
    {
        TAKEOFF_ACTIVATE,
        LANDING_ACTIVATE,
        GO_TO_ACTIVATE,
        MISSION_ACTIVATE
    };

    enum TurnMode
    {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    };

    enum FinishAction
    {
        NO_FINISH_ACTION,
        RETURN_TO_HOME,
        AUTO_LANDING,
        RETURN_TO_POINT_ZERO,
        INFINITE_MODE
    };

    enum ExecTimes
    {
        ONCE = 1,
        TWICE = 2
    };

    /*!< 0: auto mode(point to next waypoint) <br>*/
    /*!< 1: lock as an initial value <br>*/
    /*!< 2: controlled by RC <br>*/
    /*!< 3: use waypoint's yaw(tgt_yaw) */
    enum YawMode
    {
        YAW_AUTO_MODE,
        LOCK_AS_AN_INITIAL_VALUE,
        CONTROLLED_BY_RC,
        USE_WAYPOINTS_YAW
    };

    /*!< 0: point to point, after reaching the target waypoint hover,
     * complete waypoints action (if any),
     * then fly to the next waypoint <br>
     * 1: Coordinated turn mode, smooth transition between waypoints,
     * no waypoints task <br>
     */
    enum TraceMode
    {
        POINT_TO_POINT,
        COORDINATED_TURN_MODE
    };

    /*!< 0: exit waypoint and failsafe <br>*/
    /*!< 1: continue the waypoint <br>*/
    enum RcLostAction
    {
        EXIT_WAYPOINT,
        CONTINUE_WAYPOINT
    };

    /*!< 0: free mode, no control on gimbal <br>*/
    /*!< 1: auto mode, Smooth transition between waypoints <br>*/
    enum GimbalPitch
    {
        FREE_MODE,
        AUTO_MODE
    };

    struct VehicleSetpoint
    {
        base::Time time;
        base::Vector3d position;
        base::Angle heading;
    };

    struct Action
    {
        int command;
        int command_parameter;
    };

    struct Waypoint
    {
        base::Angle latitude;     /*!< Latitude (radian) */
        base::Angle longitude;    /*!< Longitude (radian) */
        float altitude;           /*!< Altitude (relative altitude from takeoff point) */
        float damping;            /*!< Bend length (effective coordinated turn mode only) */
        base::Angle yaw;          /*!< Yaw (degree) */
        base::Angle gimbal_pitch; /*!< Gimbal pitch */
        TurnMode turn_mode;      /*!< Turn mode <br> */

        int action_time_limit;
        int total_running_times;
        std::vector<Action> actions;
    };

    struct Mission
    {
        // initial waypoint settings
        float max_velocity;
        float idle_velocity;
        base::Angle latitude;
        base::Angle longitude;
        float altitude;
        FinishAction finish_action;
        ExecTimes executive_times;
        YawMode yaw_mode;
        TraceMode trace_mode;
        RcLostAction rc_lost_action;
        GimbalPitch gimbal_pitch;
        std::vector<Waypoint> waypoints;
    };
}

#endif

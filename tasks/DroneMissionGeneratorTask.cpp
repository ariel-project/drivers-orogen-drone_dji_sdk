/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneMissionGeneratorTask.hpp"

using namespace drone_dji_sdk;

DroneMissionGeneratorTask::DroneMissionGeneratorTask(std::string const &name)
    : DroneMissionGeneratorTaskBase(name)
{
}

DroneMissionGeneratorTask::~DroneMissionGeneratorTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DroneMissionGeneratorTask.hpp for more detailed
// documentation about them.

bool DroneMissionGeneratorTask::configureHook()
{
    if (!DroneMissionGeneratorTaskBase::configureHook())
        return false;

    // Create mission
    mMission.timestamp = base::Time::now();
    mMission.max_velocity = 10;
    mMission.idle_velocity = 5;
    for (int i = 0; i < 3; i++)
        mMission.position[i] = 0;
    mMission.finish_action = FinishAction::NO_FINISH_ACTION;
    mMission.executive_times = ExecTimes::ONCE;
    mMission.yaw_mode = YawMode::YAW_AUTO_MODE;
    mMission.trace_mode = TraceMode::POINT_TO_POINT;
    mMission.rc_lost_action = RcLostAction::CONTINUE_WAYPOINT;
    mMission.gimbal_pitch = GimbalPitch::FREE_MODE;

    std::vector<Waypoint> generatedWaypts =
        createWaypoints();
    mMission.waypoints = generatedWaypts;

    return true;
}

bool DroneMissionGeneratorTask::startHook()
{
    if (!DroneMissionGeneratorTaskBase::startHook())
        return false;

    return true;
}

void DroneMissionGeneratorTask::updateHook()
{
    _cmd_mission.write(mMission);

    DroneMissionGeneratorTaskBase::updateHook();
}

std::vector<Waypoint>
DroneMissionGeneratorTask::createWaypoints()
{
    // Create generic waypoint settings
    Waypoint wp;
    wp.damping = 0;
    wp.yaw.rad = 0;
    wp.gimbal_pitch.rad = 0;
    wp.turn_mode = TurnMode::CLOCKWISE;
    wp.action_time_limit = 100;
    wp.total_running_times = 0;
    Action actions;
    actions.command = 0;
    actions.command_parameter = 0;
    for (int i = 0; i < 16; i++)
        wp.actions.push_back(actions);
    // Create waypoints vector
    std::vector<Waypoint> wp_list;
    // Create polygon
    wp.position[0] = 10;
    wp.position[1] = 0;
    wp.position[2] = 8;
    wp_list.push_back(wp);
    wp.position[0] = 0;
    wp.position[1] = 10;
    wp.position[2] = 12;
    wp_list.push_back(wp);
    wp.position[0] = 0;
    wp.position[1] = -10;
    wp.position[2] = 5;
    wp_list.push_back(wp);
    return wp_list;
}

void DroneMissionGeneratorTask::errorHook()
{
    DroneMissionGeneratorTaskBase::errorHook();
}
void DroneMissionGeneratorTask::stopHook()
{
    DroneMissionGeneratorTaskBase::stopHook();
}
void DroneMissionGeneratorTask::cleanupHook()
{
    DroneMissionGeneratorTaskBase::cleanupHook();
}

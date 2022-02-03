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
    // cmd input
    base::samples::RigidBodyState gps_position;
    if (_cmd_input.read(gps_position) == RTT::NoData)
        return;

    // Create mission
    Mission mission;
    mission.max_velocity = 10;
    mission.idle_velocity = 5;
    for(int i=0;i<3;i++)
        mission.position[i] = 0;
    mission.finish_action = FinishAction::NO_FINISH_ACTION;
    mission.executive_times = ExecTimes::ONCE;
    mission.yaw_mode = YawMode::YAW_AUTO_MODE;
    mission.trace_mode = TraceMode::POINT_TO_POINT;
    mission.rc_lost_action = RcLostAction::CONTINUE_WAYPOINT;
    mission.gimbal_pitch = GimbalPitch::FREE_MODE;

    std::vector<Waypoint> generatedWaypts =
        createWaypoints(gps_position);
    mission.waypoints = generatedWaypts;

    _mission.write(mission);

    DroneMissionGeneratorTaskBase::updateHook();
}

std::vector<Waypoint>
DroneMissionGeneratorTask::createWaypoints(base::samples::RigidBodyState gps_position)
{
    // Create generic waypoint settings
    Waypoint wp;
    wp.damping = 0;
    wp.yaw.rad = 0;
    wp.gimbal_pitch.rad = 0;
    wp.turn_mode = TurnMode::CLOCKWISE;
    wp.action_time_limit = 100;
    wp.total_running_times = 0;
    for (int i = 0; i < 16; ++i)
    {
        wp.actions[i].command = 0;
        wp.actions[i].command_parameter = 0;
    }
    // Create waypoints vector
    std::vector<Waypoint> wp_list;
    // Create circle
    for (int i = 1; i < 10; i++)
    {
        wp.position[0] = sin(0.8*1000*base::Time::Milliseconds);
        wp.position[1] = cos(0.8*1000*base::Time::Milliseconds);
        wp.position[2] = 8;
        wp_list.push_back(wp);
    }
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

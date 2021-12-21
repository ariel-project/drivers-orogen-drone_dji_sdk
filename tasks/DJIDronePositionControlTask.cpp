/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DJIDronePositionControlTask.hpp"

using namespace drone_dji_sdk;

DJIDronePositionControlTask::DJIDronePositionControlTask(std::string const& name)
    : DJIDronePositionControlTaskBase(name)
{
}

DJIDronePositionControlTask::~DJIDronePositionControlTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DJIDronePositionControlTask.hpp for more detailed
// documentation about them.

bool DJIDronePositionControlTask::configureHook()
{
    if (! DJIDronePositionControlTaskBase::configureHook())
        return false;
    return true;
}
bool DJIDronePositionControlTask::startHook()
{
    if (! DJIDronePositionControlTaskBase::startHook())
        return false;
    return true;
}
void DJIDronePositionControlTask::updateHook()
{
    DJIDronePositionControlTaskBase::updateHook();
}
void DJIDronePositionControlTask::errorHook()
{
    DJIDronePositionControlTaskBase::errorHook();
}
void DJIDronePositionControlTask::stopHook()
{
    DJIDronePositionControlTaskBase::stopHook();
}
void DJIDronePositionControlTask::cleanupHook()
{
    DJIDronePositionControlTaskBase::cleanupHook();
}

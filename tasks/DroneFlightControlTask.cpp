/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"
// #include "hal/osdkhal_linux.h"
// #include "osal/osdkosal_linux.h"

using namespace DJI::OSDK;
using namespace drone_dji_sdk;
using namespace VehicleStatus;

DroneFlightControlTask::DroneFlightControlTask(std::string const &name)
    : DroneFlightControlTaskBase(name)
{
}

DroneFlightControlTask::~DroneFlightControlTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DroneFlightControlTask.hpp for more detailed
// documentation about them.

bool DroneFlightControlTask::configureHook()
{
    if (!DroneFlightControlTaskBase::configureHook())
        return false;

    // Init class members
    mFunctionTimeout = 1;  // second
    mStatusFreqInHz = 10;  // Hz

    if (!initVehicle())
        return false;
    if (!checkTelemetrySubscription())
        return false;

    return true;
}

bool DroneFlightControlTask::startHook()
{
    if (!DroneFlightControlTaskBase::startHook())
        return false;

    // Obtain Control Authority
    mVehicle->obtainCtrlAuthority(mFunctionTimeout);
    return true;
}

typedef DroneFlightControlTask::States TaskState;
static TaskState djiStatusFlightToTaskState(uint8_t status)
{
    switch (status)
    {
    case DJI::OSDK::VehicleStatus::FlightStatus::STOPED:
        return TaskState::DJI_STOPPED;
    case DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND:
        return TaskState::DJI_ON_GROUND;
    case DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR:
        return TaskState::DJI_IN_AIR;
    }
    // Never reached
    throw std::invalid_argument("invalid controller state");
}

void DroneFlightControlTask::updateHook()
{
    _pose_samples.write(getRigidBodyState());
    _battery.write(getBatteryStatus());

    // cmd input
    COMMAND_ACTION cmd_input;
    if (_cmd_input.read(cmd_input) != RTT::NoData)
        return;

    // setpoint inputs
    VehicleSetpoint cmd_pos;
    if (_cmd_pos.read(cmd_pos) != RTT::NewData)
        return;

    // Check status
    auto djiStatusFlight = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    DroneFlightControlTask::States status = djiStatusFlightToTaskState(djiStatusFlight);
    if (state() != status)
        state(status);

    switch (cmd_input)
    {
    case TAKEOFF_ACTIVATE:
        return takeoff(cmd_pos);
    case PRE_LANDING_ACTIVATE:
        return preLand(cmd_pos);
    case LANDING_ACTIVATE:
        return land();
    case GO_TO_ACTIVATE:
        return goTo(cmd_pos, getRigidBodyState());
    case MISSION_ACTIVATE:
        return mission();
    }

    DroneFlightControlTaskBase::updateHook();
}
void DroneFlightControlTask::errorHook()
{
    DroneFlightControlTaskBase::errorHook();
}
void DroneFlightControlTask::stopHook()
{
    mVehicle->releaseCtrlAuthority(mFunctionTimeout);

    DroneFlightControlTaskBase::stopHook();
}
void DroneFlightControlTask::cleanupHook()
{
    int pkgIndex = 0;
    /*! Telemetry subscription*/
    if (!teardownSubscription(pkgIndex))
        DERROR("Failed to tear down Subscription!");

    DroneFlightControlTaskBase::cleanupHook();
}

bool DroneFlightControlTask::initVehicle()
{
    bool threadSupport = true;
    bool useAdvancedSensing = false;
    mVehicle = new Vehicle(_device.get().c_str(),
                           _baudrate.get(),
                           threadSupport,
                           useAdvancedSensing);

    // Check if the communication is working fine
    if (!mVehicle->protocolLayer->getDriver()->getDeviceStatus())
    {
      std::cout << "Comms appear to be incorrectly set up. Exiting." << std::endl;
      delete (mVehicle);
      this->mVehicle = nullptr;
      return false;
    }

    // Activate
    mActivateData.ID = _app_id.get();
    char app_key[65];
    mActivateData.encKey = app_key;
    strcpy(mActivateData.encKey, _app_key.get().c_str());
    mActivateData.version = mVehicle->getFwVersion();
    ACK::ErrorCode ack   = mVehicle->activate(&mActivateData, mFunctionTimeout);

    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, __func__);
      delete (mVehicle);
      this->mVehicle = nullptr;
      return false;
    }

    return true;
}

bool DroneFlightControlTask::checkTelemetrySubscription()
{
    /*! Verify and setup the subscription */
    // Status flight, status display mode, Topic quaternion and GPS fused
    const int pkgIndex = 0;
    Telemetry::TopicName topicList[] = {Telemetry::TOPIC_STATUS_FLIGHT, Telemetry::TOPIC_STATUS_DISPLAYMODE,
                                        Telemetry::TOPIC_QUATERNION, Telemetry::TOPIC_GPS_FUSED};
    int topicSize = sizeof(topicList) / sizeof(topicList[0]);
    if (!setUpSubscription(pkgIndex, mStatusFreqInHz, topicList, topicSize))
        return false;

    /*! wait for subscription data come*/
    sleep(mFunctionTimeout);
    return true;
}

void DroneFlightControlTask::takeoff(VehicleSetpoint const &initialPoint)
{
    auto djiStatusFlight = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_ASSISTED_TAKEOFF ||
        djiDisplayMode == DisplayMode::MODE_AUTO_TAKEOFF)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        goTo(initialPoint, getRigidBodyState());
        return;
    }

    mVehicle->control->takeoff(mFunctionTimeout);
}

void DroneFlightControlTask::preLand(VehicleSetpoint const &finalPoint)
{
    auto djiStatusFlight = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_AUTO_LANDING ||
        djiDisplayMode == DisplayMode::MODE_FORCE_AUTO_LANDING)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        goTo(finalPoint, getRigidBodyState());
    }
}

void DroneFlightControlTask::land()
{
    auto djiStatusFlight = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_AUTO_LANDING ||
        djiDisplayMode == DisplayMode::MODE_FORCE_AUTO_LANDING)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        mVehicle->control->land(mFunctionTimeout);
    }
}

void DroneFlightControlTask::goTo(VehicleSetpoint const &setpoint,
                                  base::samples::RigidBodyState const &pose)
{
    // get the vector between aircraft and target point.
    base::Vector3d offset = (setpoint.position - pose.position);
    // get the orientation between aircraft and target - in Deg!
    float yawInRad = base::getYaw(pose.orientation);
    float yawDesiredInDeg = (setpoint.heading.rad - yawInRad)*180/M_PI;

    float32_t xCmd = static_cast<float>(offset[0]);
    float32_t yCmd = static_cast<float>(offset[1]);
    float32_t zCmd = static_cast<float>(setpoint.position[2]) + static_cast<float>(pose.position[2]);

    mVehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredInDeg);
}

void DroneFlightControlTask::mission()
{

}

power_base::BatteryStatus DroneFlightControlTask::getBatteryStatus() const
{
    auto djiBattery = mVehicle->broadcast->getBatteryInfo();

    power_base::BatteryStatus battery;
    battery.time = base::Time::fromMilliseconds(mVehicle->broadcast->getTimeStamp().time_ms);
    battery.current = djiBattery.current;
    battery.voltage = djiBattery.voltage;
    battery.charge = static_cast<float>(djiBattery.percentage) / 100;
    return battery;
}

base::samples::RigidBodyState DroneFlightControlTask::getRigidBodyState() const
{
    base::samples::RigidBodyState cmd;
    cmd.time = base::Time::fromMilliseconds(mVehicle->broadcast->getTimeStamp().time_ms);
    cmd.orientation.w() = mVehicle->broadcast->getQuaternion().q0;
    cmd.orientation.x() = mVehicle->broadcast->getQuaternion().q1;
    cmd.orientation.y() = mVehicle->broadcast->getQuaternion().q2;
    cmd.orientation.z() = mVehicle->broadcast->getQuaternion().q3;
    cmd.velocity.x() = mVehicle->broadcast->getVelocity().x;
    cmd.velocity.y() = mVehicle->broadcast->getVelocity().y;
    cmd.velocity.z() = mVehicle->broadcast->getVelocity().z;
    cmd.angular_velocity.x() = mVehicle->broadcast->getAngularRate().x;
    cmd.angular_velocity.y() = mVehicle->broadcast->getAngularRate().y;
    cmd.angular_velocity.z() = mVehicle->broadcast->getAngularRate().z;
    // Get GPS position information
    Telemetry::GPSInfo gpsInfo = mVehicle->broadcast->getGPSInfo();
    // Convert position data from GPS to NWU
    gps_base::Solution solution;
    solution.time = cmd.time;
    solution.latitude = gpsInfo.latitude;
    solution.longitude = gpsInfo.longitude;
    solution.altitude = gpsInfo.HFSL;
    // Get position
    gps_base::UTMConverter gpsSolution;
    base::samples::RigidBodyState gpsPosition = gpsSolution.convertToNWU(solution);
    cmd.position = gpsPosition.position;

    return cmd;
}

bool DroneFlightControlTask::setUpSubscription(int pkgIndex, int freq,
                                               Telemetry::TopicName topicList[],
                                               uint8_t topicSize)
{
    if (mVehicle)
    {
        /*! Telemetry: Verify the subscription*/
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = mVehicle->subscribe->verify(mFunctionTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        bool pkgStatus = mVehicle->subscribe->initPackageFromTopicList(
            pkgIndex, topicSize, topicList, enableTimestamp, freq);
        if (!(pkgStatus))
            return pkgStatus;

        usleep(5000);
        /*! Start listening to the telemetry data */
        subscribeStatus = mVehicle->subscribe->startPackage(pkgIndex, mFunctionTimeout);
        usleep(5000);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            /*! Cleanup*/
            ACK::ErrorCode ack = mVehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
            if (ACK::getError(ack))
            {
                DERROR("Error unsubscription; please restart the drone/FC to get "
                       "back to a clean state");
            }
            return false;
        }
        return true;
    }
    else
    {
        DERROR("vehicle haven't been initialized", __func__);
        return false;
    }
}

bool DroneFlightControlTask::teardownSubscription(const int pkgIndex)
{
    ACK::ErrorCode ack = mVehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
    if (ACK::getError(ack))
    {
        DERROR(
            "Error unsubscription; please restart the drone/FC to get back "
            "to a clean state.");
        return false;
    }
    return true;
}

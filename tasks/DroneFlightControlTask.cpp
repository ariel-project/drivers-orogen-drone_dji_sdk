/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"

using namespace DJI::OSDK;
using namespace drone_dji_sdk;
using namespace drone_control;
using namespace VehicleStatus;
using namespace base;

DroneFlightControlTask::DroneFlightControlTask(std::string const& name)
    : DroneFlightControlTaskBase(name)
{
}

DroneFlightControlTask::~DroneFlightControlTask() {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DroneFlightControlTask.hpp for more detailed
// documentation about them.

bool DroneFlightControlTask::configureHook()
{
    if (!DroneFlightControlTaskBase::configureHook())
        return false;
    // Configure GPS stuffs
    mGPSSolution.setParameters(_utm_parameters.get());
    // Get pre land distance threshold
    mPositionThreshold = _pre_land_distance_threshold.get();

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

    mLastMission = Mission();
    state(TELEMETRY);
    return true;
}

DroneFlightControlTask::States
DroneFlightControlTask::runtimeStatesTransition(Telemetry::SDKInfo control_status)
{
    DroneFlightControlTask::States current_state = state();
    switch (current_state)
    {
        case TELEMETRY:
        {
            if (_cmd_action.connected())
            {
                return CONTROLLING;
            }
            else
            {
                return TELEMETRY;
            }
        }
        case CONTROLLING:
        {
            bool can_take_ctrl = canTakeControl(control_status);
            if (!_cmd_action.connected())
            {
                return TELEMETRY;
            }
            else if (can_take_ctrl)
            {
                return CONTROLLING;
            }
            else
            {
                return CONTROL_LOST;
            }
        }
        case CONTROL_LOST:
        {
            if (_cmd_action.connected())
            {
                return CONTROL_LOST;
            }
            else
            {
                return TELEMETRY;
            }
        }
        default:
            return current_state;
    }
}

DroneFlightControlTask::States
DroneFlightControlTask::transitionToControlling()
{
    Time deadline = Time::now() + Time::fromSeconds(mFunctionTimeout);
    Telemetry::SDKInfo control_device;
    control_device = mVehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();
    auto now = Time::now();
    /** Try to obtain control authority from the Remote Controller. The timeout loop
     * is needed because there is no guarantee that a single try is enough, and the
     * feedback that this was successfull might be late.
     */
    while (!canTakeControl(control_device))
    {
        auto timeout = (deadline - now).toSeconds();
        mAuthorityStatus = mVehicle->obtainCtrlAuthority(timeout);
        ACK::getErrorCodeMessage(mAuthorityStatus, __func__);
        control_device = mVehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();
        now = Time::now();
        if (now > deadline)
        {
            return CONTROL_LOST;
        }
    }
    return CONTROLLING;
}

void DroneFlightControlTask::updateHook()
{
    Time timestamp = Time::fromMilliseconds(mVehicle->broadcast->getTimeStamp().time_ms);
    if (timestamp != mPrevDataTimestamp)
    {
        _pose_samples.write(getRigidBodyState());
        _battery.write(getBatteryStatus());
    }
    mPrevDataTimestamp = timestamp;

    mStatus.authority_status =
        static_cast<AuthorityRequestResult>(ACK::getError(mAuthorityStatus));
    auto control_device =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();

    States new_state = runtimeStatesTransition(control_device);

    if (state() != new_state)
    {
        if (new_state == CONTROLLING)
        {
            States resulting_state = transitionToControlling();
            state(resulting_state);
        }
        else
        {
            state(new_state);
        }
    }

    auto dji_flight_status = writeStatus(control_device);

    // cmd input
    CommandAction cmd_action;
    if (_cmd_action.read(cmd_action) == RTT::NoData)
    {
        return;
    }

    if (state() != CONTROLLING)
    {
        return;
    }

    // Check status
    if (dji_flight_status != VehicleStatus::FlightStatus::IN_AIR &&
        cmd_action != TAKEOFF_ACTIVATE)
    {
        return;
    }

    switch (cmd_action)
    {
        case TAKEOFF_ACTIVATE:
        {
            // setpoint input
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;
            return takeoff(setpoint);
        }
        case REACTIVE_TAKEOFF_VELOCITY_ACTIVATE:
        {
            // setpoint input
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;
            return reactiveTakeoffVelocity(setpoint);
        }
        case LANDING_ACTIVATE:
        {
            // setpoint input
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;
            return land(setpoint);
        }
        case POSITION_CONTROL_ACTIVATE:
        {
            // setpoint input
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;
            return posControl(setpoint);
        }
        case VELOCITY_CONTROL_ACTIVATE:
        {
            // setpoint input
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;
            return velControl(setpoint);
        }
        case MISSION_ACTIVATE:
        {
            // waypoint input
            Mission wypMission;
            if (_cmd_mission.read(wypMission) != RTT::NewData)
                return;
            return mission(wypMission);
        }
    }

    DroneFlightControlTaskBase::updateHook();
}
void DroneFlightControlTask::errorHook() { DroneFlightControlTaskBase::errorHook(); }
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
    std::unique_ptr<Vehicle> vehicle(new Vehicle(
        _device.get().c_str(),
        _baudrate.get(),
        threadSupport,
        useAdvancedSensing));

    // Check if the communication is working fine
    if (!vehicle->protocolLayer->getDriver()->getDeviceStatus())
    {
        DERROR("Comms appear to be incorrectly set up. Exiting.");
        return false;
    }

    // Activate
    mActivateData.ID = _app_id.get();
    char app_key[65];
    mActivateData.encKey = app_key;
    strcpy(mActivateData.encKey, _app_key.get().c_str());
    mActivateData.version = vehicle->getFwVersion();
    ACK::ErrorCode ack = vehicle->activate(&mActivateData, mFunctionTimeout);

    if (ACK::getError(ack))
    {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }

    mVehicle = std::move(vehicle);
    return true;
}

bool DroneFlightControlTask::checkTelemetrySubscription()
{
    /*! Verify and setup the subscription */
    // Status flight, status display mode, Topic quaternion and GPS fused
    const int pkgIndex = 0;
    std::vector<Telemetry::TopicName> topicList = {
        Telemetry::TOPIC_STATUS_FLIGHT,
        Telemetry::TOPIC_STATUS_DISPLAYMODE,
        Telemetry::TOPIC_QUATERNION,
        Telemetry::TOPIC_GPS_FUSED,
        Telemetry::TOPIC_CONTROL_DEVICE};
    if (!setUpSubscription(pkgIndex, mStatusFreqInHz, topicList))
        return false;

    /*! wait for subscription data come*/
    sleep(mFunctionTimeout);
    return true;
}

bool DroneFlightControlTask::missionInitSettings(Mission wypMission)
{
    // Waypoint Mission : Initialization
    WayPointInitSettings fdata = getWaypointInitDefaults(wypMission);

    ACK::ErrorCode initAck = mVehicle->missionManager->init(
        DJI_MISSION_TYPE::WAYPOINT,
        mFunctionTimeout,
        &fdata);
    if (ACK::getError(initAck))
    {
        ACK::getErrorCodeMessage(initAck, __func__);
        return false;
    }

    mVehicle->missionManager->printInfo();
    return true;
}

void DroneFlightControlTask::takeoff(VehicleSetpoint setpoint)
{
    auto djiStatusFlight =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_ASSISTED_TAKEOFF ||
        djiDisplayMode == DisplayMode::MODE_AUTO_TAKEOFF)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        posControl(setpoint);
        return;
    }

    mVehicle->control->takeoff(mFunctionTimeout);
}

void DroneFlightControlTask::reactiveTakeoffVelocity(VehicleSetpoint setpoint)
{
    auto djiStatusFlight =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_ASSISTED_TAKEOFF ||
        djiDisplayMode == DisplayMode::MODE_AUTO_TAKEOFF)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        velControl(setpoint);
        return;
    }

    mVehicle->control->takeoff(mFunctionTimeout);
}

void DroneFlightControlTask::land(VehicleSetpoint setpoint)
{
    auto djiDisplayMode =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_AUTO_LANDING ||
        djiDisplayMode == DisplayMode::MODE_FORCE_AUTO_LANDING)
    {
        return;
    }

    if (!checkDistanceThreshold(setpoint))
    {
        posControl(setpoint);
        return;
    }
    mVehicle->control->land(mFunctionTimeout);
}

bool DroneFlightControlTask::checkDistanceThreshold(VehicleSetpoint pos)
{
    // get position
    base::samples::RigidBodyState current_position = getRigidBodyState();
    // get distance
    base::Vector3d dist = pos.position - current_position.position;

    return (dist.norm() < mPositionThreshold);
}

void DroneFlightControlTask::posControl(VehicleSetpoint setpoint)
{
    // get the vector between aircraft and target point.
    base::Vector3d position = getRigidBodyState().position;
    // get offset
    base::Vector3d offset = (setpoint.position - position);
    // Convert to deg!
    float yawDesiredInDeg = -(setpoint.yaw.rad) * 180 / M_PI;

    // Convert to NEU
    float32_t xCmd = static_cast<float>(offset[0]);
    float32_t yCmd = -static_cast<float>(offset[1]);
    float32_t zCmd = static_cast<float>(setpoint.position[2]);

    mVehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredInDeg);
}

void DroneFlightControlTask::velControl(VehicleSetpoint setpoint)
{
    // get the vector between aircraft and target point.
    base::Vector3d velocity = getRigidBodyState().velocity;
    // get offset
    base::Vector3d offset = (setpoint.velocity - velocity);
    // Convert to deg!
    float yawRateDesiredInDeg = -setpoint.yaw_rate * 180 / M_PI;

    // Convert to NEU
    float32_t xCmd = static_cast<float>(offset[0]);
    float32_t yCmd = -static_cast<float>(offset[1]);
    float32_t zCmd = static_cast<float>(setpoint.velocity[2]);

    mVehicle->control->velocityAndYawRateCtrl(xCmd, yCmd, zCmd, yawRateDesiredInDeg);
}

void DroneFlightControlTask::mission(Mission wypMission)
{
    // Check if there is a new mission, with new waypoints, actions etc.
    if (mLastMission == wypMission)
        return;
    mLastMission = wypMission;

    // Config mission
    if (!missionInitSettings(wypMission))
        return;

    std::vector<base::Vector3d> positions;
    for (unsigned int i = 0; i < wypMission.waypoints.size(); i++)
    {
        WayPointSettings wpp = getWaypointSettings(wypMission.waypoints[i], i);
        positions.push_back({wpp.latitude, wpp.longitude, wpp.altitude});
        ACK::WayPointIndex wpDataACK =
            mVehicle->missionManager->wpMission->uploadIndexData(&wpp, mFunctionTimeout);
        ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
    }
    // debug
    _cmd_out_position.write(positions);

    // Waypoint Mission: Start
    ACK::ErrorCode startAck =
        mVehicle->missionManager->wpMission->start(mFunctionTimeout);
    if (ACK::getError(startAck))
    {
        ACK::getErrorCodeMessage(startAck, __func__);
    }
    else
    {
        DSTATUS("Starting Waypoint Mission..");
    }
}

WayPointInitSettings DroneFlightControlTask::getWaypointInitDefaults(Mission mission)
{
    WayPointInitSettings fdata;
    fdata.indexNumber = mission.waypoints.size();
    fdata.maxVelocity = mission.max_velocity;
    fdata.idleVelocity = mission.idle_velocity;
    fdata.finishAction = mission.finish_action;
    fdata.executiveTimes = mission.executive_times;
    fdata.yawMode = mission.yaw_mode;
    fdata.traceMode = mission.trace_mode;
    fdata.RCLostAction = mission.rc_lost_action;
    fdata.gimbalPitch = mission.gimbal_pitch;
    fdata.latitude = mission.position[0];
    fdata.longitude = mission.position[1];
    fdata.altitude = mission.position[2];

    return fdata;
}

WayPointSettings
DroneFlightControlTask::getWaypointSettings(Waypoint cmd_waypoint, int index)
{
    // Convert local position cmd to lat/long
    gps_base::Solution pos = convertToGPSPosition(cmd_waypoint);

    WayPointSettings wp;
    wp.index = index;
    // Convert to rad
    wp.latitude = pos.latitude * M_PI / 180;
    wp.longitude = pos.longitude * M_PI / 180;
    wp.altitude = pos.altitude;
    wp.damping = cmd_waypoint.damping;
    // convert to degree (sorry for the mess, complain with dji)
    wp.yaw = cmd_waypoint.yaw.getDeg();
    wp.gimbalPitch = cmd_waypoint.gimbal_pitch.getDeg();
    wp.turnMode = cmd_waypoint.turn_mode;
    for (int i = 0; i < 8; i++)
        wp.reserved[i] = 0;
    if (cmd_waypoint.actions.empty())
        wp.hasAction = 0;
    else
        wp.hasAction = 1;
    wp.actionTimeLimit = 100;
    wp.actionNumber = cmd_waypoint.actions.size();
    wp.actionRepeat = 0;
    for (int i = 0; i < 16; i++)
    {
        wp.commandList[i] = cmd_waypoint.actions[i].command;
        wp.commandParameter[i] = cmd_waypoint.actions[i].command_parameter;
    }
    return wp;
}

gps_base::Solution DroneFlightControlTask::convertToGPSPosition(Waypoint cmd_waypoint)
{
    base::samples::RigidBodyState cmd;
    cmd.position = cmd_waypoint.position;
    base::samples::RigidBodyState utm = mGPSSolution.convertNWUToUTM(cmd);
    gps_base::Solution gps = mGPSSolution.convertUTMToGPS(utm);

    return gps;
}

power_base::BatteryStatus DroneFlightControlTask::getBatteryStatus() const
{
    auto djiBattery = mVehicle->broadcast->getBatteryInfo();

    power_base::BatteryStatus battery;
    battery.time =
        base::Time::fromMilliseconds(mVehicle->broadcast->getTimeStamp().time_ms);
    battery.current = djiBattery.current;
    battery.voltage = djiBattery.voltage;
    battery.charge = static_cast<float>(djiBattery.percentage) / 100;
    return battery;
}

base::samples::RigidBodyState DroneFlightControlTask::getRigidBodyState() const
{
    // convert everything do NWU (Rock convention)
    base::samples::RigidBodyState cmd;
    cmd.time = base::Time::now();
    Telemetry::Quaternion orientation = mVehicle->broadcast->getQuaternion();
    Eigen::Quaterniond q_bodyned2ned(
        orientation.q0,
        orientation.q1,
        orientation.q2,
        orientation.q3);
    Eigen::Quaterniond q_ned2nwu =
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond q_bodynwu2nwu = q_ned2nwu * q_bodyned2ned * q_ned2nwu.conjugate();
    cmd.orientation = q_bodynwu2nwu;
    cmd.velocity.x() = mVehicle->broadcast->getVelocity().x;
    cmd.velocity.y() = -mVehicle->broadcast->getVelocity().y;
    cmd.velocity.z() = mVehicle->broadcast->getVelocity().z;
    cmd.angular_velocity.x() = mVehicle->broadcast->getAngularRate().x;
    cmd.angular_velocity.y() = -mVehicle->broadcast->getAngularRate().y;
    cmd.angular_velocity.z() = -mVehicle->broadcast->getAngularRate().z;
    // Get GPS position information
    Telemetry::GlobalPosition gpsInfo = mVehicle->broadcast->getGlobalPosition();
    gps_base::Solution solution;
    solution.time = cmd.time;
    // Convert to degree - Solution expect the value in degree
    solution.latitude = gpsInfo.latitude * 180 / M_PI;
    solution.longitude = gpsInfo.longitude * 180 / M_PI;
    solution.altitude = gpsInfo.height;
    // Convert position data from GPS to NWU
    cmd.position = mGPSSolution.convertToNWU(solution).position;

    return cmd;
}

bool DroneFlightControlTask::setUpSubscription(
    int pkgIndex,
    int freq,
    std::vector<Telemetry::TopicName> topicList)
{
    if (!mVehicle)
    {
        DERROR("vehicle haven't been initialized", __func__);
        return false;
    }
    /*! Telemetry: Verify the subscription*/
    ACK::ErrorCode subscribeStatus = mVehicle->subscribe->verify(mFunctionTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    bool enableTimestamp = false;
    bool pkgStatus = mVehicle->subscribe->initPackageFromTopicList(
        pkgIndex,
        topicList.size(),
        topicList.data(),
        enableTimestamp,
        freq);
    if (!(pkgStatus))
        return false;

    /*! Start listening to the telemetry data */
    subscribeStatus = mVehicle->subscribe->startPackage(pkgIndex, mFunctionTimeout);

    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        /*! Cleanup*/
        ACK::ErrorCode ack =
            mVehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
        if (ACK::getError(ack))
        {
            DERROR("Error unsubscription; please restart the drone/FC to get "
                   "back to a clean state");
            throw std::invalid_argument("Error unsubscription");
        }
        return false;
    }
    return true;
}

bool DroneFlightControlTask::teardownSubscription(const int pkgIndex)
{
    ACK::ErrorCode ack = mVehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
    if (ACK::getError(ack))
    {
        DERROR("Error unsubscription; please restart the drone/FC to get back "
               "to a clean state.");
        return false;
    }
    return true;
}

uint8_t DroneFlightControlTask::writeStatus(Telemetry::SDKInfo const& control_device)
{
    auto dji_flight_status =
        mVehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    mStatus.control_device = static_cast<ControlDevice>(control_device.deviceStatus);
    mStatus.device_flight_status =
        static_cast<DeviceFlightStatus>(control_device.flightStatus);
    _status.write(mStatus);
    drone_control::FlightStatus flight_status =
        static_cast<drone_control::FlightStatus>(dji_flight_status);
    _flight_status.write(flight_status);
    return dji_flight_status;
}

bool DroneFlightControlTask::canTakeControl(Telemetry::SDKInfo const& control_device)
{
    /** This check whether the SDK is controlling the drone or if the control device
     * changes to the Remote Controller, and outputs the state.
     *
     * 0 - RC
     * 1 - Mobile app
     * 2 - Serial
     */
    writeStatus(control_device);
    return control_device.deviceStatus == 2;
}
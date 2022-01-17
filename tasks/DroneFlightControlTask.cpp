/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"

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
    mControlFreqInHz = 50; // Hz
    mStatusFreqInHz = 10;  // Hz

    // Setup environment and init vehicle
    mSetup = Setup(false); // AdvancedSensing = false
    if (mSetup.vehicle == NULL)
        DSTATUS("Vehicle not initialized, exiting.");
    setupEnvironment();
    if (!initVehicle())
        return false;
    if (!checkTelemetrySubscription())
        return false;

    // Init controller and set joystick mode
    setupController();

    // init mission settings and register
    if (initMissionSetting() != ErrorCode::SysCommonErr::Success)
        return false;
    // Register mission events and states
    mSetup.vehicle->waypointV2Mission->RegisterMissionEventCallback(mSetup.vehicle->waypointV2Mission,
                                                                    updateMissionEvent);
    mSetup.vehicle->waypointV2Mission->RegisterMissionStateCallback(mSetup.vehicle->waypointV2Mission,
                                                                    updateMissionState);
    return true;
}

bool DroneFlightControlTask::startHook()
{
    if (!DroneFlightControlTaskBase::startHook())
        return false;

    // Obtain Control Authority
    mSetup.vehicle->flightController->obtainJoystickCtrlAuthorityAsync(obtainJoystickCtrlAuthorityCB,
                                                                       nullptr,
                                                                       mFunctionTimeout,
                                                                       2);
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
    auto djiStatusFlight = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
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
    mSetup.vehicle->flightController->releaseJoystickCtrlAuthorityAsync(releaseJoystickCtrlAuthorityCB,
                                                                        nullptr,
                                                                        mFunctionTimeout,
                                                                        2);
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

void DroneFlightControlTask::setupEnvironment()
{
    /*
     * Setup environment
     */
    static T_OsdkLoggerConsole printConsole = {
        OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        DroneFlightControlTask::OsdkUser_Console,
    };
    if (DJI_REG_LOGGER_CONSOLE(&printConsole) != true)
        throw std::runtime_error("logger console register fail");

    static T_OsdkHalUartHandler halUartHandler = {
        DroneFlightControlTask::OsdkLinux_UartInit,
        DroneFlightControlTask::OsdkLinux_UartSendData,
        DroneFlightControlTask::OsdkLinux_UartReadData,
        DroneFlightControlTask::OsdkLinux_UartClose,
    };
    if (DJI_REG_UART_HANDLER(&halUartHandler) != true)
        throw std::runtime_error("Uart handler register fail");

    static T_OsdkOsalHandler osalHandler = {
        DroneFlightControlTask::OsdkLinux_TaskCreate,
        DroneFlightControlTask::OsdkLinux_TaskDestroy,
        DroneFlightControlTask::OsdkLinux_TaskSleepMs,
        DroneFlightControlTask::OsdkLinux_MutexCreate,
        DroneFlightControlTask::OsdkLinux_MutexDestroy,
        DroneFlightControlTask::OsdkLinux_MutexLock,
        DroneFlightControlTask::OsdkLinux_MutexUnlock,
        DroneFlightControlTask::OsdkLinux_SemaphoreCreate,
        DroneFlightControlTask::OsdkLinux_SemaphoreDestroy,
        DroneFlightControlTask::OsdkLinux_SemaphoreWait,
        DroneFlightControlTask::OsdkLinux_SemaphoreTimedWait,
        DroneFlightControlTask::OsdkLinux_SemaphorePost,
        DroneFlightControlTask::OsdkLinux_GetTimeMs,
        DroneFlightControlTask::OsdkLinux_Malloc,
        DroneFlightControlTask::OsdkLinux_Free,
    };
    if (DJI_REG_OSAL_HANDLER(&osalHandler) != true)
        throw std::runtime_error("Osal handler register fail");
}

void DroneFlightControlTask::setupController()
{
    mFlightController = new FlightController(mSetup.vehicle);
    FlightController::JoystickMode joystickMode = {
        FlightController::HorizontalLogic::HORIZONTAL_POSITION,
        FlightController::VerticalLogic::VERTICAL_POSITION,
        FlightController::YawLogic::YAW_ANGLE,
        FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
        FlightController::StableMode::STABLE_ENABLE,
    };
    mSetup.vehicle->flightController->setJoystickMode(joystickMode);
}

bool DroneFlightControlTask::initVehicle()
{
    ACK::ErrorCode ack;

    /*! Linker initialization */
    if (!mSetup.initLinker())
    {
        DERROR("Failed to initialize Linker");
        return false;
    }
    /*! Linker add uart channel */
    if (!mSetup.addFCUartChannel(_device.get().c_str(),
                                 _baudrate.get()))
    {
        DERROR("Failed to initialize Linker channel");
        return false;
    }
    /*! Linker add USB acm channel */
    if (!mSetup.addUSBACMChannel(_acm_port.get().c_str(),
                                 _baudrate.get()))
    {
        DERROR("Failed to initialize ACM Linker channel!");
        return false;
    }
    /*! Vehicle initialization */
    if (!mSetup.linker)
    {
        DERROR("Linker get failed.");
        mSetup.vehicle = nullptr;
        return false;
    }
    mSetup.vehicle = new Vehicle(mSetup.linker);
    if (!mSetup.vehicle)
    {
        DERROR("Vehicle create failed.");

        delete (mSetup.vehicle);
        mSetup.vehicle = nullptr;
        return false;
    }
    // Activate
    mActivateData.ID = _app_id.get();
    char app_key[65];
    mActivateData.encKey = app_key;
    strcpy(mActivateData.encKey, _app_key.get().c_str());
    mActivateData.version = mSetup.vehicle->getFwVersion();
    ack = mSetup.vehicle->activate(&mActivateData, mFunctionTimeout);
    if (ACK::getError(ack))
    {
        ACK::getErrorCodeMessage(ack, __func__);
        delete (mSetup.vehicle);
        mSetup.vehicle = nullptr;
        return false;
    }
    if (!mSetup.vehicle->isM300())
        mSetup.vehicle->setUSBFlightOn(true);

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
    // auto djiStatusFlight = getTelemetryValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    // auto djiDisplayMode = getTelemetryValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
    auto djiStatusFlight = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

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

    mSetup.vehicle->flightController->startTakeoffAsync(
        startAsyncCmdCallBack, (UserData) "start to takeoff");
}

void DroneFlightControlTask::preLand(VehicleSetpoint const &finalPoint)
{
    // auto djiStatusFlight = getTelemetryValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    // auto djiDisplayMode = getTelemetryValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
    auto djiStatusFlight = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

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
    // auto djiStatusFlight = getTelemetryValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    // auto djiDisplayMode = getTelemetryValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
    auto djiStatusFlight = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
    auto djiDisplayMode = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (djiDisplayMode == DisplayMode::MODE_AUTO_LANDING ||
        djiDisplayMode == DisplayMode::MODE_FORCE_AUTO_LANDING)
    {
        return;
    }

    if (djiStatusFlight == VehicleStatus::FlightStatus::IN_AIR)
    {
        mSetup.vehicle->flightController->startLandingAsync(startAsyncCmdCallBack,
                                                            (UserData) "start to landing");
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

    Telemetry::Vector3f positionCommand;
    positionCommand.x = static_cast<float>(offset[0]);
    positionCommand.y = static_cast<float>(offset[1]);
    FlightController::JoystickCommand joystickCommand = {
        positionCommand.x, positionCommand.y,
        static_cast<float>(setpoint.position[2]) + static_cast<float>(pose.position[2]),
        yawDesiredInDeg};

    mSetup.vehicle->flightController->setJoystickCommand(joystickCommand);
    mSetup.vehicle->flightController->joystickAction();
}

power_base::BatteryStatus DroneFlightControlTask::getBatteryStatus() const
{
    auto djiBattery = mSetup.vehicle->broadcast->getBatteryInfo();

    power_base::BatteryStatus battery;
    battery.time = base::Time::fromMilliseconds(mSetup.vehicle->broadcast->getTimeStamp().time_ms);
    battery.current = djiBattery.current;
    battery.voltage = djiBattery.voltage;
    battery.charge = static_cast<float>(djiBattery.percentage) / 100;
    return battery;
}

base::samples::RigidBodyState DroneFlightControlTask::getRigidBodyState() const
{
    base::samples::RigidBodyState cmd;
    cmd.time = base::Time::fromMilliseconds(mSetup.vehicle->broadcast->getTimeStamp().time_ms);
    cmd.orientation.w() = mSetup.vehicle->broadcast->getQuaternion().q0;
    cmd.orientation.x() = mSetup.vehicle->broadcast->getQuaternion().q1;
    cmd.orientation.y() = mSetup.vehicle->broadcast->getQuaternion().q2;
    cmd.orientation.z() = mSetup.vehicle->broadcast->getQuaternion().q3;
    cmd.velocity.x() = mSetup.vehicle->broadcast->getVelocity().x;
    cmd.velocity.y() = mSetup.vehicle->broadcast->getVelocity().y;
    cmd.velocity.z() = mSetup.vehicle->broadcast->getVelocity().z;
    cmd.angular_velocity.x() = mSetup.vehicle->broadcast->getAngularRate().x;
    cmd.angular_velocity.y() = mSetup.vehicle->broadcast->getAngularRate().y;
    cmd.angular_velocity.z() = mSetup.vehicle->broadcast->getAngularRate().z;
    // Get GPS position information
    Telemetry::GPSInfo gpsInfo = mSetup.vehicle->broadcast->getGPSInfo();
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

void DroneFlightControlTask::startAsyncCmdCallBack(ErrorCode::ErrorCodeType retCode,
                                                   UserData SampleLog)
{
    DSTATUS("retCode : 0x%lX", retCode);
    if (retCode == ErrorCode::SysCommonErr::Success)
        DSTATUS("Pass : %s.", SampleLog);
    else
    {
        DERROR("Error : %s. Error code : %d", SampleLog, retCode);
        ErrorCode::printErrorCodeMsg(retCode);
    }
}

bool DroneFlightControlTask::setUpSubscription(int pkgIndex, int freq,
                                               Telemetry::TopicName topicList[],
                                               uint8_t topicSize)
{
    if (mSetup.vehicle)
    {
        /*! Telemetry: Verify the subscription*/
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = mSetup.vehicle->subscribe->verify(mFunctionTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        bool enableTimestamp = false;
        bool pkgStatus = mSetup.vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, topicSize, topicList, enableTimestamp, freq);
        if (!(pkgStatus))
            return pkgStatus;

        usleep(5000);
        /*! Start listening to the telemetry data */
        subscribeStatus = mSetup.vehicle->subscribe->startPackage(pkgIndex, mFunctionTimeout);
        usleep(5000);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            /*! Cleanup*/
            ACK::ErrorCode ack = mSetup.vehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
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
    ACK::ErrorCode ack = mSetup.vehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
    if (ACK::getError(ack))
    {
        DERROR(
            "Error unsubscription; please restart the drone/FC to get back "
            "to a clean state.");
        return false;
    }
    return true;
}

void DroneFlightControlTask::mission()
{
    if (!mSetup.vehicle->isM300())
    {
        DSTATUS("This sample only supports M300!");
        return;
    }

    GetRemainRamAck actionMemory = {0};
    ErrorCode::ErrorCodeType ret;

    /*! upload mission */
    /*! upload mission's timeout need to be longer than 2s*/
    int uploadMissionTimeOut = 3;
    ret = uploadWaypointMission(uploadMissionTimeOut);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("upload waypoint mission error");
        return;
    }
    sleep(mFunctionTimeout);

    /*! download mission */
    std::vector<WaypointV2> mission;
    ret = downloadWaypointMission(mission);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("download waypoint mission error");
        return;
    }
    sleep(mFunctionTimeout);

    /*! upload  actions */
    /*! check action memory */
    ret = getActionRemainMemory(actionMemory);
    if (actionMemory.remainMemory <= 0)
    {
        DSTATUS("action memory is not enough.Can not upload more action!");
        return;
    }

    ret = uploadWapointActions();
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("upload waypoint action error");
        return;
    }

    ret = getActionRemainMemory(actionMemory);
    sleep(mFunctionTimeout);

    /*! start mission */
    ret = startWaypointMission();
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("start waypoint mission error");
        return;
    }
    sleep(20);

    /*! set global cruise speed */
    setGlobalCruiseSpeed(1.5);
    sleep(mFunctionTimeout);

    /*! get global cruise speed */
    getGlobalCruiseSpeed();
    sleep(mFunctionTimeout);

    /*! pause the mission*/
    ret = pauseWaypointMission();
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("pause waypoint mission error");
        return;
    }
    sleep(5);

    /*! resume the mission*/
    ret = resumeWaypointMission();
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("resume waypoint mission error");
        return;
    }
    sleep(50);
}

ErrorCode::ErrorCodeType DroneFlightControlTask::initMissionSetting()
{

    uint16_t polygonNum = 6;
    float32_t radius = 6;

    uint16_t actionNum = 5;
    srand(int(time(0)));

    /*! Generate waypoints*/

    /*! Generate actions*/
    mActions = generateWaypointActions(actionNum);

    /*! Init waypoint settings*/
    WayPointV2InitSettings missionInitSettings;
    missionInitSettings.missionID = rand();
    missionInitSettings.repeatTimes = 1;
    missionInitSettings.finishedAction = DJIWaypointV2MissionFinishedGoHome;
    missionInitSettings.maxFlightSpeed = 10;
    missionInitSettings.autoFlightSpeed = 2;
    missionInitSettings.exitMissionOnRCSignalLost = 1;
    missionInitSettings.gotoFirstWaypointMode = DJIWaypointV2MissionGotoFirstWaypointModePointToPoint;
    missionInitSettings.mission = generatePolygonWaypoints(radius, polygonNum);
    missionInitSettings.missTotalLen = missionInitSettings.mission.size();

    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->init(&missionInitSettings,
                                                                           mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Init mission setting ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Init mission setting successfully!");

    return ret;
}

std::vector<DJIWaypointV2Action> DroneFlightControlTask::generateWaypointActions(uint16_t actionNum)
{
    std::vector<DJIWaypointV2Action> actionVector;

    for (uint16_t i = 0; i < actionNum; i++)
    {
        DJIWaypointV2SampleReachPointTriggerParam sampleReachPointTriggerParam;
        sampleReachPointTriggerParam.waypointIndex = i;
        sampleReachPointTriggerParam.terminateNum = 0;

        auto trigger = DJIWaypointV2Trigger(DJIWaypointV2ActionTriggerTypeSampleReachPoint, &sampleReachPointTriggerParam);
        auto cameraActuatorParam = DJIWaypointV2CameraActuatorParam(DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);
        auto actuator = DJIWaypointV2Actuator(DJIWaypointV2ActionActuatorTypeCamera, 0, &cameraActuatorParam);
        auto action = DJIWaypointV2Action(i, trigger, actuator);
        actionVector.push_back(action);
    }
    return actionVector;
}

std::vector<WaypointV2> DroneFlightControlTask::generatePolygonWaypoints(float32_t radius,
                                                                         uint16_t polygonNum)
{
    // Let's create a vector to store our waypoints in.
    std::vector<WaypointV2> waypointList;
    WaypointV2 startPoint;
    WaypointV2 waypointV2;

    Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type
        subscribeGPosition = mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    startPoint.latitude = subscribeGPosition.latitude;
    startPoint.longitude = subscribeGPosition.longitude;
    startPoint.relativeHeight = 15;
    setWaypointV2Defaults(startPoint);
    waypointList.push_back(startPoint);

    // Iterative algorithm
    for (int i = 0; i < polygonNum; i++)
    {
        float32_t angle = i * 2 * M_PI / polygonNum;
        setWaypointV2Defaults(waypointV2);
        float32_t X = radius * cos(angle);
        float32_t Y = radius * sin(angle);
        waypointV2.latitude = X / EARTH_RADIUS + startPoint.latitude;
        waypointV2.longitude = Y / (EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
        waypointV2.relativeHeight = startPoint.relativeHeight;
        waypointList.push_back(waypointV2);
    }
    waypointList.push_back(startPoint);
    return waypointList;
}

void DroneFlightControlTask::setWaypointV2Defaults(WaypointV2 &waypointV2)
{

    waypointV2.waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
    waypointV2.headingMode = DJIWaypointV2HeadingModeAuto;
    waypointV2.config.useLocalCruiseVel = 0;
    waypointV2.config.useLocalMaxVel = 0;

    waypointV2.dampingDistance = 40;
    waypointV2.heading = 0;
    waypointV2.turnMode = DJIWaypointV2TurnModeClockwise;

    waypointV2.pointOfInterest.positionX = 0;
    waypointV2.pointOfInterest.positionY = 0;
    waypointV2.pointOfInterest.positionZ = 0;
    waypointV2.maxFlightSpeed = 9;
    waypointV2.autoFlightSpeed = 2;
}

void DroneFlightControlTask::obtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode,
                                                           UserData userData)
{
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess)
        DSTATUS("ObtainJoystickCtrlAuthoritySuccess");
}

void DroneFlightControlTask::releaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode,
                                                            UserData userData)
{
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess)
        DSTATUS("ReleaseJoystickCtrlAuthoritySuccess");
}

ErrorCode::ErrorCodeType DroneFlightControlTask::uploadWaypointMission(int timeout)
{
    //  ErrorCode::ErrorCodeType ret = vehiclePtr->waypointV2Mission->uploadMission(this->mission,timeout);
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->uploadMission(mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Upload waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Upload waypoint v2 mission successfully!");

    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::downloadWaypointMission(std::vector<WaypointV2> &mission)
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->downloadMission(mission, mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Download waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Download waypoint v2 mission successfully!");

    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::getActionRemainMemory(GetRemainRamAck &actionMemory)
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->getActionRemainMemory(actionMemory,
                                                                                            mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("get waypoint v2 action remain memory failed:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("get waypoint v2 action remain memory successfully!");

    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::uploadWapointActions()
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->uploadAction(mActions,
                                                                                   mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Upload waypoint v2 actions ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Upload waypoint v2 actions successfully!");

    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::startWaypointMission()
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->start(mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Start waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Start waypoint v2 mission successfully!");

    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::stopWaypointMission()
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->stop(mFunctionTimeout);
    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::pauseWaypointMission()
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->pause(mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Pause waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Pause waypoint v2 mission successfully!");

    sleep(5);
    return ret;
}

ErrorCode::ErrorCodeType DroneFlightControlTask::resumeWaypointMission()
{

    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->resume(mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Resume Waypoint v2 mission ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return ret;
    }
    else
        DSTATUS("Resume Waypoint v2 mission successfully!");

    return ret;
}

void DroneFlightControlTask::getGlobalCruiseSpeed()
{
    GlobalCruiseSpeed cruiseSpeed = 0;
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed,
                                                                                           mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Get glogal cruise speed failed ErrorCode:0x%lX", ret);
        ErrorCode::printErrorCodeMsg(ret);
        return;
    }
    DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
}

void DroneFlightControlTask::setGlobalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed)
{
    ErrorCode::ErrorCodeType ret = mSetup.vehicle->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed,
                                                                                           mFunctionTimeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX", cruiseSpeed, ret);
        ErrorCode::printErrorCodeMsg(ret);
        return;
    }
    DSTATUS("Current cruise speed is: %f m/s", cruiseSpeed);
}

//10HZ push ;1HZ print
E_OsdkStat
DroneFlightControlTask::updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                           const uint8_t *cmdData, void *userData)
{
    if (cmdInfo)
    {
        if (userData)
        {
            auto *wp2Ptr = (WaypointV2MissionOperator *)userData;
            auto *missionStatePushAck =
                (DJI::OSDK::MissionStatePushAck *)cmdData;

            wp2Ptr->setCurrentState(wp2Ptr->getCurrentState());
            wp2Ptr->setCurrentState((DJI::OSDK::DJIWaypointV2MissionState)missionStatePushAck->data.state);
            static uint32_t curMs = 0;
            static uint32_t preMs = 0;
            OsdkOsal_GetTimeMs(&curMs);
            if (curMs - preMs >= 1000)
            {
                preMs = curMs;
                DSTATUS("missionStatePushAck->commonDataVersion:%d", missionStatePushAck->commonDataVersion);
                DSTATUS("missionStatePushAck->commonDataLen:%d", missionStatePushAck->commonDataLen);
                DSTATUS("missionStatePushAck->data.state:0x%x", missionStatePushAck->data.state);
                DSTATUS("missionStatePushAck->data.curWaypointIndex:%d", missionStatePushAck->data.curWaypointIndex);
                DSTATUS("missionStatePushAck->data.velocity:%d", missionStatePushAck->data.velocity);
            }
        }
        else
            DERROR("cmdInfo is a null value");

        return OSDK_STAT_OK;
    }
    return OSDK_STAT_ERR_ALLOC;
}

/*! only push 0x00,0x10,0x11 event*/
E_OsdkStat
DroneFlightControlTask::updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                           const uint8_t *cmdData, void *userData)
{
    if (cmdInfo)
    {
        if (userData)
        {
            auto *MissionEventPushAck =
                (DJI::OSDK::MissionEventPushAck *)cmdData;

            DSTATUS("MissionEventPushAck->event ID :0x%x", MissionEventPushAck->event);

            if (MissionEventPushAck->event == 0x01)
                DSTATUS("interruptReason:0x%x", MissionEventPushAck->data.interruptReason);
            if (MissionEventPushAck->event == 0x02)
                DSTATUS("recoverProcess:0x%x", MissionEventPushAck->data.recoverProcess);
            if (MissionEventPushAck->event == 0x03)
                DSTATUS("finishReason:0x%x", MissionEventPushAck->data.finishReason);

            if (MissionEventPushAck->event == 0x10)
                DSTATUS("current waypointIndex:%d", MissionEventPushAck->data.waypointIndex);

            if (MissionEventPushAck->event == 0x11)
            {
                DSTATUS("currentMissionExecNum:%d", MissionEventPushAck->data.MissionExecEvent.currentMissionExecNum);
            }

            return OSDK_STAT_OK;
        }
    }
    return OSDK_STAT_SYS_ERR;
}

E_OsdkStat
DroneFlightControlTask::OsdkUser_Console(const uint8_t *data,
                                         uint16_t dataLen)
{
    printf("%s", data);
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartInit(const char *port,
                                           const int baudrate,
                                           T_HalObj *obj)
{
    int st_baud[] = {B4800, B9600, B19200, B38400, B57600, B115200,
                     B230400, B460800, B921600, B1000000, B1152000, B3000000};
    int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
                      230400, 460800, 921600, 1000000, 1152000, 3000000};

    struct termios options;
    E_OsdkStat OsdkStat = OSDK_STAT_OK;
    long unsigned int i = 0;

    if (!port)
        return OSDK_STAT_ERR_PARAM;

    obj->uartObject.fd = open(port, O_RDWR | O_NOCTTY);
    if (obj->uartObject.fd == -1)
    {
        OsdkStat = OSDK_STAT_ERR;
        return OsdkStat;
    }

    if (tcgetattr(obj->uartObject.fd, &options) != 0)
    {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;

        return OsdkStat;
    }

    for (i = 0; i < sizeof(std_rate) / sizeof(int); ++i)
    {
        if (std_rate[i] == baudrate)
        {
            /* set standard baudrate */
            cfsetispeed(&options, st_baud[i]);
            cfsetospeed(&options, st_baud[i]);
            break;
        }
    }
    if (i == sizeof(std_rate) / sizeof(int))
    {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;

        return OsdkStat;
    }

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_iflag &= ~INPCK;
    options.c_cflag &= ~CSTOPB;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    tcflush(obj->uartObject.fd, TCIFLUSH);

    if (tcsetattr(obj->uartObject.fd, TCSANOW, &options) != 0)
    {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;

        return OsdkStat;
    }

    return OsdkStat;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartSendData(const T_HalObj *obj,
                                               const uint8_t *pBuf,
                                               uint32_t bufLen)
{
    uint32_t realLen;

    if ((obj == NULL) || (obj->uartObject.fd == -1))
        return OSDK_STAT_ERR;

    realLen = write(obj->uartObject.fd, pBuf, bufLen);
    if (realLen == bufLen)
        return OSDK_STAT_OK;
    else
        return OSDK_STAT_ERR;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartReadData(const T_HalObj *obj,
                                               uint8_t *pBuf,
                                               uint32_t *bufLen)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1))
        return OSDK_STAT_ERR;

    ssize_t readLen = read(obj->uartObject.fd, pBuf, 1024);
    if (readLen < 0)
    {
        *bufLen = 0;
        printf("errno = %d\n", errno);
        perror("OsdkLinux_UartReadData");
    }
    else
        *bufLen = readLen;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartClose(T_HalObj *obj)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1))
        return OSDK_STAT_ERR;

    close(obj->uartObject.fd);

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_TaskCreate(
    T_OsdkTaskHandle *task,
    void *(*taskFunc)(void *),
    uint32_t stackSize, void *arg)
{
    int result;
    // *task = malloc(sizeof(pthread_t));
    result = pthread_create((pthread_t *)task, NULL, taskFunc, arg);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_TaskDestroy(T_OsdkTaskHandle task)
{
    pthread_cancel(*(pthread_t *)task);
    pthread_join(*(pthread_t *)task, NULL);

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_TaskSleepMs(uint32_t time_ms)
{
    usleep(1000 * time_ms);
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexCreate(T_OsdkMutexHandle *mutex)
{
    int result;
    // *mutex = malloc(sizeof(pthread_mutex_t));
    result = pthread_mutex_init((pthread_mutex_t *)mutex, NULL);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_destroy((pthread_mutex_t *)mutex);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexLock(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_lock((pthread_mutex_t *)mutex);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex)
{
    int result = pthread_mutex_unlock((pthread_mutex_t *)mutex);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreCreate(
    T_OsdkSemHandle *semaphore,
    uint32_t initValue)
{
    int result;
    // *semaphore = malloc(sizeof(sem_t));
    result = sem_init((sem_t *)semaphore, 0, (unsigned int)initValue);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreDestroy(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_destroy((sem_t *)semaphore);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreWait(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_wait((sem_t *)semaphore);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreTimedWait(
    T_OsdkSemHandle semaphore,
    uint32_t waitTime)
{
    int result;
    struct timespec semaphoreWaitTime;
    struct timeval systemTime;
    gettimeofday(&systemTime, NULL);
    systemTime.tv_usec += waitTime * 1000;
    if (systemTime.tv_usec >= 1000000)
    {
        systemTime.tv_sec += systemTime.tv_usec / 1000000;
        systemTime.tv_usec %= 1000000;
    }
    semaphoreWaitTime.tv_sec = systemTime.tv_sec;
    semaphoreWaitTime.tv_nsec = systemTime.tv_usec * 1000;
    result = sem_timedwait((sem_t *)semaphore, &semaphoreWaitTime);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphorePost(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_post((sem_t *)semaphore);
    if (result != 0)
        return OSDK_STAT_ERR;

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_GetTimeMs(uint32_t *ms)
{
    struct timeval time;
    gettimeofday(&time, NULL);
    *ms = (time.tv_sec * 1000 + time.tv_usec / 1000);

    return OSDK_STAT_OK;
}
void *DroneFlightControlTask::OsdkLinux_Malloc(uint32_t size)
{
    return malloc(size);
}
void DroneFlightControlTask::OsdkLinux_Free(void *ptr)
{
    free(ptr);
}

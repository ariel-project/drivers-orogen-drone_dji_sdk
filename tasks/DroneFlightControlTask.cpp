/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"

using namespace DJI::OSDK;
using namespace drone_dji_sdk;

static const double EarthCenter = 6378137.0;
static const double DEG2RAD = 0.01745329252;

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
    // AdvancedSensing = false
    mSetup = Setup(false);
    if (mSetup.vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
    }
    mFunctionTimeout = 1; // second
    mPosThresholdInM = 0.8;
    mYawThresholdInDeg = 1;
    setupEnvironment();
    if (!initVehicle())
        return false;
    if (!checkTelemetrySubscription())
        return false;

    // Obtain Control Authority
    mSetup.vehicle->flightController->obtainJoystickCtrlAuthorityAsync(obtainJoystickCtrlAuthorityCB,
                                                                       nullptr,
                                                                       mFunctionTimeout,
                                                                       2);

    return true;
}

bool DroneFlightControlTask::startHook()
{
    if (!DroneFlightControlTaskBase::startHook())
        return false;
    return true;
}

typedef DroneFlightControlTask::States TaskState;
static TaskState checkState(uint8_t status)
{
    switch (status)
    {
    case 0:
        return TaskState::DJI_STOPPED;
    case 1:
        return TaskState::DJI_ON_GROUND;
    case 2:
        return TaskState::DJI_IN_AIR;
    }
    // Never reached
    throw std::invalid_argument("invalid controller state");
}

void DroneFlightControlTask::updateHook()
{
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
    {
        throw std::runtime_error("logger console register fail");
    }
    static T_OsdkHalUartHandler halUartHandler = {
        DroneFlightControlTask::OsdkLinux_UartInit,
        DroneFlightControlTask::OsdkLinux_UartSendData,
        DroneFlightControlTask::OsdkLinux_UartReadData,
        DroneFlightControlTask::OsdkLinux_UartClose,
    };
    if (DJI_REG_UART_HANDLER(&halUartHandler) != true)
    {
        throw std::runtime_error("Uart handler register fail");
    }
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
    {
        throw std::runtime_error("Osal handler register fail");
    }
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
        // if (mSetup.vehicle) delete (mSetup.vehicle);
        mSetup.vehicle = nullptr;
        return false;
    }
    mSetup.vehicle = new Vehicle(mSetup.linker);
    if (!mSetup.vehicle)
    {
        DERROR("Vehicle create failed.");
        if (mSetup.vehicle)
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
        if (mSetup.vehicle)
            delete (mSetup.vehicle);
        mSetup.vehicle = nullptr;
        return false;
    }
    if (!mSetup.vehicle->isM300())
    {
        mSetup.vehicle->setUSBFlightOn(true);
    }
    return true;
}

bool DroneFlightControlTask::checkTelemetrySubscription()
{
    /*! Verify and setup the subscription */
    // Status flight and status display mode
    const int pkgIndex = 0;
    int freq = 10;
    Telemetry::TopicName topicList_1[] = {Telemetry::TOPIC_STATUS_FLIGHT, Telemetry::TOPIC_STATUS_DISPLAYMODE};
    int topicSize = sizeof(topicList_1) / sizeof(topicList_1[0]);
    if (!setUpSubscription(pkgIndex, freq, topicList_1, topicSize))
    {
        return false;
    }
    // Topic quaternion and GPS fused
    int controlFreqInHz = 50; // Hz
    Telemetry::TopicName topicList_2[] = {Telemetry::TOPIC_QUATERNION, Telemetry::TOPIC_GPS_FUSED};
    int numTopic = sizeof(topicList_2) / sizeof(topicList_2[0]);
    if (!setUpSubscription(pkgIndex, controlFreqInHz, topicList_2, numTopic))
    {
        return false;
    }
    return true;
}

bool DroneFlightControlTask::monitoredTakeoff()
{
    int pkgIndex = 0;
    //! Start takeoff
    //ErrorCode::ErrorCodeType takeoffStatus =
    mSetup.vehicle->flightController->startTakeoffSync(mFunctionTimeout);

    //! Motors start check
    if (!motorStartedCheck())
    {
        std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
        teardownSubscription(pkgIndex);
        return false;
    }
    else
    {
        std::cout << "Motors spinning...\n";
    }
    //! In air check
    if (!takeOffInAirCheck())
    {
        std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                     "motors are spinning."
                  << std::endl;
        teardownSubscription(pkgIndex);
        return false;
    }
    else
    {
        std::cout << "Ascending...\n";
    }

    //! Finished takeoff check
    if (takeoffFinishedCheck())
    {
        std::cout << "Successful takeoff!\n";
    }
    else
    {
        std::cout << "Takeoff finished, but the aircraft is in an unexpected mode. "
                     "Please connect DJI GO.\n";
        teardownSubscription(pkgIndex);
        return false;
    }
    teardownSubscription(pkgIndex);
    return true;
}

bool DroneFlightControlTask::monitoredLanding()
{
    int pkgIndex = 0;
    /*! Start landing */
    DSTATUS("Start landing action");
    ErrorCode::ErrorCodeType landingErrCode =
        mSetup.vehicle->flightController->startLandingSync(mFunctionTimeout);
    if (landingErrCode != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Fail to execute landing action! Error code: "
               "%llx\n ",
               landingErrCode);
        return false;
    }

    /*! Step 3: check Landing start*/
    if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING))
    {
        DERROR("Fail to execute Landing action!");
        return false;
    }
    else
    {
        /*! Step 4: check Landing finished*/
        if (this->landFinishedCheck())
        {
            DSTATUS("Successful landing!");
        }
        else
        {
            DERROR("Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI Assistant.");
            teardownSubscription(pkgIndex);
            return false;
        }
    }

    /*! Step 5: Cleanup */
    teardownSubscription(pkgIndex);
    return true;
}

bool DroneFlightControlTask::moveByPositionOffset(const Telemetry::Vector3f &offsetDesired,
                                                  float yawDesiredInDeg)
{
    int timeoutInMilSec = 40000;
    int controlFreqInHz = 50; // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;   // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs; // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;
    int pkgIndex = 0;

    /* now we need position-height broadcast to obtain the real-time altitude of the aircraft, 
   * which is consistent with the altitude closed-loop data of flight control internal position control
   * TO DO:the data will be replaced by new data subscription.
   */
    if (!startGlobalPositionBroadcast())
    {
        return false;
    }
    sleep(1);

    //! get origin position and relative height(from home point)of aircraft.
    Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type originGPSPosition =
        mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    Telemetry::GlobalPosition currentBroadcastGP = mSetup.vehicle->broadcast->getGlobalPosition();
    float32_t originHeightBaseHomepoint = currentBroadcastGP.height;
    FlightController::JoystickMode joystickMode = {
        FlightController::HorizontalLogic::HORIZONTAL_POSITION,
        FlightController::VerticalLogic::VERTICAL_POSITION,
        FlightController::YawLogic::YAW_ANGLE,
        FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
        FlightController::StableMode::STABLE_ENABLE,
    };
    mSetup.vehicle->flightController->setJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec)
    {
        Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type currentGPSPosition =
            mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
        Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type currentQuaternion =
            mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
        currentBroadcastGP = mSetup.vehicle->broadcast->getGlobalPosition();
        float yawInRad = quaternionToEulerAngle(currentQuaternion).z;
        //! get the vector between aircraft and origin point.

        Telemetry::Vector3f localOffset = localOffsetFromGpsAndFusedHeightOffset(currentGPSPosition, originGPSPosition,
                                                                                 currentBroadcastGP.height, originHeightBaseHomepoint);
        //! get the vector between aircraft and target point.
        Telemetry::Vector3f offsetRemaining = vector3FSub(offsetDesired, localOffset);

        Telemetry::Vector3f positionCommand = offsetRemaining;
        horizCommandLimit(speedFactor, positionCommand.x, positionCommand.y);

        FlightController::JoystickCommand joystickCommand = {
            positionCommand.x, positionCommand.y,
            offsetDesired.z + originHeightBaseHomepoint, yawDesiredInDeg};

        mSetup.vehicle->flightController->setJoystickCommand(joystickCommand);

        mSetup.vehicle->flightController->joystickAction();

        if (vectorNorm(offsetRemaining) < mPosThresholdInM &&
            std::fabs(yawInRad / DEG2RAD - yawDesiredInDeg) < mYawThresholdInDeg)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        }
        else
        {
            if (withinBoundsCounter != 0)
            {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit)
        {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
        {
            break;
        }
        usleep(cycleTimeInMs * 1000);
        elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
        //! TODO: remove emergencyBrake
        mSetup.vehicle->flightController->emergencyBrakeAction();
        usleep(cycleTimeInMs * 1000);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        teardownSubscription(pkgIndex);
        return false;
    }
    teardownSubscription(pkgIndex);
    return true;
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
        {
            return pkgStatus;
        }

        /*! Start listening to the telemetry data */
        subscribeStatus = mSetup.vehicle->subscribe->startPackage(pkgIndex, mFunctionTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            /*! Cleanup*/
            ACK::ErrorCode ack = mSetup.vehicle->subscribe->removePackage(pkgIndex, mFunctionTimeout);
            if (ACK::getError(ack))
            {
                DERROR(
                    "Error unsubscription; please restart the drone/FC to get "
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

bool DroneFlightControlTask::motorStartedCheck()
{
    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    while (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
               VehicleStatus::FlightStatus::ON_GROUND &&
           mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
               VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
        motorsNotStarted++;
        usleep(100000);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
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

bool DroneFlightControlTask::takeOffInAirCheck()
{
    int stillOnGround = 0;
    int timeoutCycles = 110;
    while (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() !=
               VehicleStatus::FlightStatus::IN_AIR &&
           (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
        stillOnGround++;
        usleep(100000);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

bool DroneFlightControlTask::takeoffFinishedCheck()
{
    while (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
        sleep(1);
    }
    return ((mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_P_GPS) ||
            (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ATTITUDE))
               ? true
               : false;
}

bool DroneFlightControlTask::checkActionStarted(uint8_t mode)
{
    int actionNotStarted = 0;
    int timeoutCycles = 20;
    while (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() != mode &&
           actionNotStarted < timeoutCycles)
    {
        actionNotStarted++;
        Platform::instance().taskSleepMs(100);
    }
    if (actionNotStarted == timeoutCycles)
    {
        DERROR("Start actions mode %d failed, current DISPLAYMODE is: %d ...", mode,
               mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>());
        return false;
    }
    else
    {
        DSTATUS("DISPLAYMODE: %d ...", mode);
        return true;
    }
}

bool DroneFlightControlTask::landFinishedCheck(void)
{
    while (mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR)
    {
        Platform::instance().taskSleepMs(1000);
    }

    return ((mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                 VehicleStatus::DisplayMode::MODE_P_GPS ||
             mSetup.vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>() !=
                 VehicleStatus::DisplayMode::MODE_ATTITUDE))
               ? true
               : false;
}

Telemetry::Vector3f DroneFlightControlTask::vector3FSub(const Telemetry::Vector3f &vectorA,
                                                        const Telemetry::Vector3f &vectorB)
{
    Telemetry::Vector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
}

void DroneFlightControlTask::horizCommandLimit(float speedFactor, float &commandX,
                                               float &commandY)
{
    if (fabs(commandX) > speedFactor)
        commandX = signOfData<float>(commandX) * speedFactor;
    if (fabs(commandY) > speedFactor)
        commandY = signOfData<float>(commandY) * speedFactor;
}

Telemetry::Vector3f DroneFlightControlTask::localOffsetFromGpsAndFusedHeightOffset(
    const Telemetry::GPSFused &target, const Telemetry::GPSFused &origin,
    const float32_t &targetHeight, const float32_t &originHeight)
{
    Telemetry::Vector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * EarthCenter;
    deltaNed.y = deltaLon * EarthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;
    return deltaNed;
}

Telemetry::Vector3f DroneFlightControlTask::quaternionToEulerAngle(
    const Telemetry::Quaternion &quat)
{
    Telemetry::Vector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}

template <typename Type>
int DroneFlightControlTask::signOfData(Type type)
{
    return type < 0 ? -1 : 1;
}

float32_t DroneFlightControlTask::vectorNorm(Telemetry::Vector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

bool DroneFlightControlTask::startGlobalPositionBroadcast()
{
    uint8_t freq[16];

    /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
    freq[0] = DataBroadcast::FREQ_HOLD;
    freq[1] = DataBroadcast::FREQ_HOLD;
    freq[2] = DataBroadcast::FREQ_HOLD;
    freq[3] = DataBroadcast::FREQ_HOLD;
    freq[4] = DataBroadcast::FREQ_HOLD;
    freq[5] = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
    freq[6] = DataBroadcast::FREQ_HOLD;
    freq[7] = DataBroadcast::FREQ_HOLD;
    freq[8] = DataBroadcast::FREQ_HOLD;
    freq[9] = DataBroadcast::FREQ_HOLD;
    freq[10] = DataBroadcast::FREQ_HOLD;
    freq[11] = DataBroadcast::FREQ_HOLD;
    freq[12] = DataBroadcast::FREQ_HOLD;
    freq[13] = DataBroadcast::FREQ_HOLD;

    ACK::ErrorCode ack = mSetup.vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack))
    {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    else
    {
        return true;
    }
}

void DroneFlightControlTask::obtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ObtainJoystickCtrlAuthoritySuccess)
    {
        DSTATUS("ObtainJoystickCtrlAuthoritySuccess");
    }
}

void DroneFlightControlTask::releaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode, UserData userData)
{
    if (errorCode == ErrorCode::FlightControllerErr::SetControlParam::ReleaseJoystickCtrlAuthoritySuccess)
    {
        DSTATUS("ReleaseJoystickCtrlAuthoritySuccess");
    }
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
    {
        return OSDK_STAT_ERR_PARAM;
    }

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
    {
        return OSDK_STAT_ERR;
    }

    realLen = write(obj->uartObject.fd, pBuf, bufLen);
    if (realLen == bufLen)
    {
        return OSDK_STAT_OK;
    }
    else
    {
        return OSDK_STAT_ERR;
    }
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartReadData(const T_HalObj *obj,
                                               uint8_t *pBuf,
                                               uint32_t *bufLen)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1))
    {
        return OSDK_STAT_ERR;
    }
    ssize_t readLen = read(obj->uartObject.fd, pBuf, 1024);
    if (readLen < 0)
    {
        *bufLen = 0;
        printf("errno = %d\n", errno);
        perror("OsdkLinux_UartReadData");
    }
    else
    {
        *bufLen = readLen;
    }

    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartClose(T_HalObj *obj)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1))
    {
        return OSDK_STAT_ERR;
    }
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
    {
        return OSDK_STAT_ERR;
    }
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
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_destroy((pthread_mutex_t *)mutex);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexLock(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_lock((pthread_mutex_t *)mutex);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex)
{
    int result = pthread_mutex_unlock((pthread_mutex_t *)mutex);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
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
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreDestroy(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_destroy((sem_t *)semaphore);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphoreWait(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_wait((sem_t *)semaphore);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
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
    {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_SemaphorePost(
    T_OsdkSemHandle semaphore)
{
    int result;
    result = sem_post((sem_t *)semaphore);
    if (result != 0)
    {
        return OSDK_STAT_ERR;
    }
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

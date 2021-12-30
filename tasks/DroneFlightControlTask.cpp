/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"

using namespace DJI::OSDK::Telemetry;
using namespace DJI::OSDK;
using namespace drone_dji_sdk;

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
    return true;
}
bool DroneFlightControlTask::startHook()
{
    if (!DroneFlightControlTaskBase::startHook())
        return false;

    // AdvancedSensing = false
    mSetup = Setup(false);
    mFunctionTimeout = 1; // second
    setupEnvironment();
    initVehicle();
    return true;
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

bool DroneFlightControlTask::monitoredTakeoff(Vehicle *vehicle, int timeout)
{
    char func[50];
    int pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }
        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex = 0;
        int freq = 10;
        TopicName topicList10Hz[] = {TOPIC_STATUS_FLIGHT,
                                     TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;
        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start takeoff
    ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
    if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(takeoffStatus, func);
        return false;
    }
    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
                   VehicleStatus::FlightStatus::ON_GROUND &&
               vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                   VehicleStatus::DisplayMode::MODE_ENGINE_START &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(100000);
        }
        if (motorsNotStarted == timeoutCycles)
        {
            std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
            // Cleanup
            if (!vehicle->isM100() && !vehicle->isLegacyM600())
            {
                vehicle->subscribe->removePackage(0, timeout);
            }
            return false;
        }
        else
        {
            std::cout << "Motors spinning...\n";
        }
    }
    else if (vehicle->isLegacyM600())
    {
        while ((vehicle->broadcast->getStatus().flight <
                DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(100000);
        }
        if (motorsNotStarted < timeoutCycles)
        {
            std::cout << "Successful TakeOff!" << std::endl;
        }
    }
    else // M100
    {
        while ((vehicle->broadcast->getStatus().flight <
                DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(100000);
        }
        if (motorsNotStarted < timeoutCycles)
        {
            std::cout << "Successful TakeOff!" << std::endl;
        }
    }
    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles = 110;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
                   VehicleStatus::FlightStatus::IN_AIR &&
               (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(100000);
        }
        if (stillOnGround == timeoutCycles)
        {
            std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                         "motors are spinning."
                      << std::endl;
            // Cleanup
            if (!vehicle->isM100() && !vehicle->isLegacyM600())
            {
                vehicle->subscribe->removePackage(0, timeout);
            }
            return false;
        }
        else
        {
            std::cout << "Ascending...\n";
        }
    }
    else if (vehicle->isLegacyM600())
    {
        while ((vehicle->broadcast->getStatus().flight <
                DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(100000);
        }
        if (stillOnGround < timeoutCycles)
        {
            std::cout << "Aircraft in air!" << std::endl;
        }
    }
    else // M100
    {
        while ((vehicle->broadcast->getStatus().flight !=
                DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(100000);
        }
        if (stillOnGround < timeoutCycles)
        {
            std::cout << "Aircraft in air!" << std::endl;
        }
    }
    // Final check: Finished takeoff
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                   VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
               vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                   VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
        {
            sleep(1);
        }
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_P_GPS ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                    VehicleStatus::DisplayMode::MODE_ATTITUDE)
            {
                std::cout << "Successful takeoff!\n";
            }
            else
            {
                std::cout
                    << "Takeoff finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.\n";
                vehicle->subscribe->removePackage(0, timeout);
                return false;
            }
        }
    }
    else
    {
        float32_t delta;
        Telemetry::GlobalPosition currentHeight;
        Telemetry::GlobalPosition deltaHeight =
            vehicle->broadcast->getGlobalPosition();
        do
        {
            sleep(4);
            currentHeight = vehicle->broadcast->getGlobalPosition();
            delta = fabs(currentHeight.altitude - deltaHeight.altitude);
            deltaHeight.altitude = currentHeight.altitude;
        } while (delta >= 0.009);
        std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
    }
    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
        }
    }
    return true;
}

bool DroneFlightControlTask::monitoredLanding(Vehicle *vehicle, int timeout)
{
    char func[50];
    int pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex = 0;
        int freq = 10;
        TopicName topicList10Hz[] = {TOPIC_STATUS_FLIGHT,
                                     TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(landingStatus, func);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles = 20;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                   VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }
    else if (vehicle->isM100())
    {
        while (vehicle->broadcast->getStatus().flight !=
                   DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }

    if (landingNotStarted == timeoutCycles)
    {
        std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            // Cleanup before return
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                             "back to a clean state.\n";
            }
        }
        return false;
    }
    else
    {
        std::cout << "Landing...\n";
    }

    // Second check: Finished landing
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                   VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
                   VehicleStatus::FlightStatus::IN_AIR)
        {
            sleep(1);
        }

        if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            std::cout << "Successful landing!\n";
        }
        else
        {
            std::cout
                << "Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                             "back to a clean state.\n";
            }
            return false;
        }
    }
    else if (vehicle->isLegacyM600())
    {
        while (vehicle->broadcast->getStatus().flight >
               DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                << "Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }
    else // M100
    {
        while (vehicle->broadcast->getStatus().flight ==
               DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                << "Landing finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }

    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
        }
    }

    return true;
}

bool DroneFlightControlTask::moveByPositionOffset(Vehicle *vehicle,
                                                  float xOffsetDesired,
                                                  float yOffsetDesired,
                                                  float zOffsetDesired,
                                                  float yawDesired,
                                                  float posThresholdInM,
                                                  float yawThresholdInDeg)
{
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    int responseTimeout = 1;
    int timeoutInMilSec = 40000;
    int controlFreqInHz = 50; // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;  // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    char func[50];

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
        // Hz
        pkgIndex = 0;
        int freq = 50;
        TopicName topicList50Hz[] = {TOPIC_QUATERNION, TOPIC_GPS_FUSED};
        int numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus =
            vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            return false;
        }

        // Also, since we don't have a source for relative height through subscription,
        // start using broadcast height
        if (!startGlobalPositionBroadcast(vehicle))
        {
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            return false;
        }
    }

    // Wait for data to come in
    sleep(1);

    // Get data

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        originSubscriptionGPS = currentSubscriptionGPS;
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void *>(&currentSubscriptionGPS),
                                 static_cast<void *>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for zCmd
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        originBroadcastGP = currentBroadcastGP;
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void *>(&currentBroadcastGP),
                                 static_cast<void *>(&originBroadcastGP));
    }

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - localOffset.z;

    // Conversions
    double yawDesiredRad = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z / DEG2RAD;
    }
    else
    {
        broadcastQ = vehicle->broadcast->getQuaternion();
        yawInRad = toEulerAngle((static_cast<void *>(&broadcastQ))).z / DEG2RAD;
    }

    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;
    float xCmd, yCmd, zCmd;

    /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
    if (xOffsetDesired > 0)
        xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
    else if (xOffsetDesired < 0)
        xCmd =
            (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
    else
        xCmd = 0;

    if (yOffsetDesired > 0)
        yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
    else if (yOffsetDesired < 0)
        yCmd =
            (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
    else
        yCmd = 0;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
    }
    else
    {
        zCmd = currentBroadcastGP.height + zOffsetDesired;
    }

    //! Main closed-loop receding setpoint position control
    while (elapsedTimeInMs < timeoutInMilSec)
    {
        vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                             yawDesiredRad / DEG2RAD);

        usleep(cycleTimeInMs * 1000);
        elapsedTimeInMs += cycleTimeInMs;
        //! Get current position in required coordinates and units
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
            yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z;
            currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
            localOffsetFromGpsOffset(vehicle, localOffset,
                                     static_cast<void *>(&currentSubscriptionGPS),
                                     static_cast<void *>(&originSubscriptionGPS));

            // Get the broadcast GP since we need the height for zCmd
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        }
        else
        {
            broadcastQ = vehicle->broadcast->getQuaternion();
            yawInRad = toEulerAngle((static_cast<void *>(&broadcastQ))).z;
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            localOffsetFromGpsOffset(vehicle, localOffset,
                                     static_cast<void *>(&currentBroadcastGP),
                                     static_cast<void *>(&originBroadcastGP));
        }

        //! See how much farther we have to go
        xOffsetRemaining = xOffsetDesired - localOffset.x;
        yOffsetRemaining = yOffsetDesired - localOffset.y;
        zOffsetRemaining = zOffsetDesired - localOffset.z;

        //! See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < speedFactor)
        {
            xCmd = xOffsetRemaining;
        }
        if (std::abs(yOffsetRemaining) < speedFactor)
        {
            yCmd = yOffsetRemaining;
        }

        if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
            std::abs(yOffsetRemaining) < posThresholdInM &&
            std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        }
        else if (std::abs(xOffsetRemaining) < posThresholdInM &&
                 std::abs(yOffsetRemaining) < posThresholdInM &&
                 std::abs(zOffsetRemaining) < posThresholdInM &&
                 std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
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
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (brakeCounter < withinControlBoundsTimeReqmt)
        {
            vehicle->control->emergencyBrake();
            usleep(cycleTimeInMs * 10);
            brakeCounter += cycleTimeInMs;
        }
    }

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            ACK::ErrorCode ack =
                vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                             "back to a clean state.\n";
            }
        }
        return ACK::FAIL;
    }

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack =
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack))
        {
            std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
        }
    }

    return ACK::SUCCESS;
}

void DroneFlightControlTask::localOffsetFromGpsOffset(Vehicle *vehicle,
                                                      Vector3f &deltaNed,
                                                      void *target,
                                                      void *origin)
{
    Telemetry::GPSFused *subscriptionTarget;
    Telemetry::GPSFused *subscriptionOrigin;
    Telemetry::GlobalPosition *broadcastTarget;
    Telemetry::GlobalPosition *broadcastOrigin;
    double deltaLon;
    double deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionTarget = (Telemetry::GPSFused *)target;
        subscriptionOrigin = (Telemetry::GPSFused *)origin;
        deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }
    else
    {
        broadcastTarget = (Telemetry::GlobalPosition *)target;
        broadcastOrigin = (Telemetry::GlobalPosition *)origin;
        deltaLon = broadcastTarget->longitude - broadcastOrigin->longitude;
        deltaLat = broadcastTarget->latitude - broadcastOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
        deltaNed.z = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
}

Vector3f DroneFlightControlTask::toEulerAngle(void *quaternionData)
{
    Telemetry::Vector3f ans;
    Telemetry::Quaternion *quaternion = (Telemetry::Quaternion *)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0 = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
        +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
        -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
        +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}

bool DroneFlightControlTask::startGlobalPositionBroadcast(Vehicle *vehicle)
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

    ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
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

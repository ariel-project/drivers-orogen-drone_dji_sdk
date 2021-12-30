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

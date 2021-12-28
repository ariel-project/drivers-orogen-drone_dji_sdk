/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DroneFlightControlTask.hpp"

using namespace drone_dji_sdk;
using namespace DJI::OSDK;


DroneFlightControlTask::DroneFlightControlTask(std::string const& name)
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
    if (! DroneFlightControlTaskBase::configureHook())
        return false;
    return true;
}
bool DroneFlightControlTask::startHook()
{
    if (! DroneFlightControlTaskBase::startHook())
        return false;
    return true;
}
void DroneFlightControlTask::updateHook()
{
    /*
     * Setup environment
     */
    static T_OsdkLoggerConsole printConsole = {
        OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,
        DroneFlightControlTask::OsdkUser_Console,
    };
    if(DJI_REG_LOGGER_CONSOLE(&printConsole) != true) {
        throw std::runtime_error("logger console register fail");
    }
    static T_OsdkHalUartHandler halUartHandler = {
        DroneFlightControlTask::OsdkLinux_UartInit,
        DroneFlightControlTask::OsdkLinux_UartSendData,
        DroneFlightControlTask::OsdkLinux_UartReadData,
        DroneFlightControlTask::OsdkLinux_UartClose,
    };
    if(DJI_REG_UART_HANDLER(&halUartHandler) != true) {
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
    if(DJI_REG_OSAL_HANDLER(&osalHandler) != true) {
        throw std::runtime_error("Osal handler register fail");
  }
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
    int st_baud[] = {B4800,   B9600,   B19200,  B38400,   B57600,   B115200,
                   B230400, B460800, B921600, B1000000, B1152000, B3000000};
    int std_rate[] = {4800,   9600,   19200,  38400,   57600,   115200,
                      230400, 460800, 921600, 1000000, 1152000, 3000000};
    
    struct termios options;
    E_OsdkStat OsdkStat = OSDK_STAT_OK;
    int i = 0;
    
    if (!port) {
        return OSDK_STAT_ERR_PARAM;
    }
    
    obj->uartObject.fd = open(port, O_RDWR | O_NOCTTY);
    if (obj->uartObject.fd == -1) {
        OsdkStat = OSDK_STAT_ERR;
        goto out;
    }
    
    if (tcgetattr(obj->uartObject.fd, &options) != 0) {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;
    
        goto out;
    }
    
    for (i = 0; i < sizeof(std_rate) / sizeof(int); ++i) {
      if (std_rate[i] == baudrate) {
        /* set standard baudrate */
        cfsetispeed(&options, st_baud[i]);
        cfsetospeed(&options, st_baud[i]);
        break;
      }
    }
    if (i == sizeof(std_rate) / sizeof(int)) {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;

        goto out;
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
    
    if (tcsetattr(obj->uartObject.fd, TCSANOW, &options) != 0) {
        close(obj->uartObject.fd);
        OsdkStat = OSDK_STAT_ERR;
    
        goto out;
    }
    out:

    return OsdkStat;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartSendData(const T_HalObj *obj,
                                               const uint8_t *pBuf,
                                               uint32_t bufLen)
{
    int32_t realLen;

    if ((obj == NULL) || (obj->uartObject.fd == -1)) {
        return OSDK_STAT_ERR;
    }

    realLen = write(obj->uartObject.fd, pBuf, bufLen);
    if (realLen == bufLen) {
        return OSDK_STAT_OK;
    } else {
        return OSDK_STAT_ERR;
    }
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartReadData(const T_HalObj *obj,
                                               uint8_t *pBuf,
                                               uint32_t *bufLen)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1)) {
        return OSDK_STAT_ERR;
    }
    ssize_t readLen = read(obj->uartObject.fd, pBuf, 1024);
    if (readLen < 0) {
        *bufLen = 0;
        printf("errno = %d\n", errno);
        perror("OsdkLinux_UartReadData");
    } else {
        *bufLen = readLen;
    }

    return OSDK_STAT_OK;
}                                
E_OsdkStat
DroneFlightControlTask::OsdkLinux_UartClose(T_HalObj *obj)
{
    if ((obj == NULL) || (obj->uartObject.fd == -1)) {
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
    if (result != 0) {
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
    if (result != 0) {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_destroy((pthread_mutex_t *)mutex);
    if (result != 0) {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexLock(T_OsdkMutexHandle mutex)
{
    int result;
    result = pthread_mutex_lock((pthread_mutex_t *)mutex);
    if (result != 0) {
        return OSDK_STAT_ERR;
    }
    return OSDK_STAT_OK;
}
E_OsdkStat
DroneFlightControlTask::OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex)
{
    int result = pthread_mutex_unlock((pthread_mutex_t *)mutex);
    if (result != 0) {
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
    if (result != 0) {
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
    if (result != 0) {
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
    if (result != 0) {
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
    if (systemTime.tv_usec >= 1000000) {
        systemTime.tv_sec += systemTime.tv_usec / 1000000;
        systemTime.tv_usec %= 1000000;
    }
    semaphoreWaitTime.tv_sec = systemTime.tv_sec;
    semaphoreWaitTime.tv_nsec = systemTime.tv_usec * 1000;
    result = sem_timedwait((sem_t *)semaphore, &semaphoreWaitTime);
    if (result != 0) {
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
    if (result != 0) {
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
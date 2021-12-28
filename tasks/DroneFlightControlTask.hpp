/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP
#define DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP

#include "drone_dji_sdk/DroneFlightControlTaskBase.hpp"
#include <dji_setup_helpers.hpp>
#include "dji_linker.hpp"
#include "dji_vehicle.hpp"
#include <pthread.h>
#include <semaphore.h>
#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "osdk_typedef.h"
#include "osdk_platform.h"


namespace drone_dji_sdk{

    /*! \class DroneFlightControlTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','drone_dji_sdk::DroneFlightControlTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class DroneFlightControlTask : public DroneFlightControlTaskBase
    {
	    friend class DroneFlightControlTaskBase;
    protected:


    public:
        /** TaskContext constructor for DroneFlightControlTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        DroneFlightControlTask(std::string const& name = "drone_dji_sdk::DroneFlightControlTask");

        /** Default deconstructor of DroneFlightControlTask
         */
	~DroneFlightControlTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

    private:

        uint32_t mFunctionTimeout;
        Vehicle::ActivateData mActivateData;
        DJI::OSDK::Setup mSetup;

        void setupEnvironment();
        bool initVehicle();

        static E_OsdkStat
        OsdkUser_Console(const uint8_t *data,
                         uint16_t dataLen);
        static E_OsdkStat
        OsdkLinux_UartInit(const char *port,
                           const int baudrate,
                           T_HalObj *obj);
        static E_OsdkStat
        OsdkLinux_UartSendData(const T_HalObj *obj,
                               const uint8_t *pBuf,
                               uint32_t bufLen);
        static E_OsdkStat
        OsdkLinux_UartReadData(const T_HalObj *obj,
                               uint8_t *pBuf,
                               uint32_t *bufLen);
        static E_OsdkStat
        OsdkLinux_UartClose(T_HalObj *obj);
        static E_OsdkStat
        OsdkLinux_TaskCreate(T_OsdkTaskHandle *task,
                             void *(*taskFunc)(void *),
                             uint32_t stackSize, void *arg);
        static E_OsdkStat
        OsdkLinux_TaskDestroy(T_OsdkTaskHandle task);
        static E_OsdkStat
        OsdkLinux_TaskSleepMs(uint32_t time_ms);
        static E_OsdkStat
        OsdkLinux_MutexCreate(T_OsdkMutexHandle *mutex);
        static E_OsdkStat
        OsdkLinux_MutexDestroy(T_OsdkMutexHandle mutex);
        static E_OsdkStat
        OsdkLinux_MutexLock(T_OsdkMutexHandle mutex);
        static E_OsdkStat
        OsdkLinux_MutexUnlock(T_OsdkMutexHandle mutex);
        static E_OsdkStat
        OsdkLinux_SemaphoreCreate(T_OsdkSemHandle *semaphore,
                                  uint32_t initValue);
        static E_OsdkStat
        OsdkLinux_SemaphoreDestroy(T_OsdkSemHandle semaphore);
        static E_OsdkStat
        OsdkLinux_SemaphoreWait(T_OsdkSemHandle semaphore);
        static E_OsdkStat
        OsdkLinux_SemaphoreTimedWait(T_OsdkSemHandle semaphore,
                                    uint32_t waitTime);
        static E_OsdkStat
        OsdkLinux_SemaphorePost(T_OsdkSemHandle semaphore);
        static E_OsdkStat
        OsdkLinux_GetTimeMs(uint32_t *ms);
        static void
        *OsdkLinux_Malloc(uint32_t size);
        static void
        OsdkLinux_Free(void *ptr);
    };
}
#endif

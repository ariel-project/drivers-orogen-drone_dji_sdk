/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP
#define DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP

#include "drone_dji_sdk/DroneFlightControlTaskBase.hpp"
#include "drone_dji_sdkTypes.hpp"
#include <gps_base/UTMConverter.hpp>
#include <gps_base/BaseTypes.hpp>
#include <dji_setup_helpers.hpp>
#include "dji_telemetry.hpp"
#include "dji_vehicle.hpp"
#include "dji_status.hpp"
#include "dji_linker.hpp"
#include "osdk_platform.h"
#include "osdk_typedef.h"
#include <pthread.h>
#include <cmath>
#include <semaphore.h>
#include "stdint.h"
#include <stdlib.h>
#include <stdexcept>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace drone_dji_sdk
{

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
        DroneFlightControlTask(std::string const &name = "drone_dji_sdk::DroneFlightControlTask");

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
        int mControlFreqInHz;
        int mStatusFreqInHz;
        Vehicle::ActivateData mActivateData;
        DJI::OSDK::Setup mSetup;
        DJI::OSDK::FlightController* mFlightController;
        std::vector<DJIWaypointV2Action> mActions;

        void land();
        void preLand(VehicleSetpoint const &finalPoint);
        void takeoff(VehicleSetpoint const &initialPoint);
        void goTo(VehicleSetpoint const &setpoint,
                  base::samples::RigidBodyState const &pose);
        void mission();

        ErrorCode::ErrorCodeType initMissionSetting();
        ErrorCode::ErrorCodeType uploadWapointActions();
        ErrorCode::ErrorCodeType startWaypointMission();
        ErrorCode::ErrorCodeType stopWaypointMission();
        ErrorCode::ErrorCodeType pauseWaypointMission();
        ErrorCode::ErrorCodeType resumeWaypointMission();
        ErrorCode::ErrorCodeType uploadWaypointMission(int timeout);
        ErrorCode::ErrorCodeType downloadWaypointMission(std::vector<WaypointV2> &mission);
        ErrorCode::ErrorCodeType getActionRemainMemory(GetRemainRamAck &actionMemory);
        std::vector<DJIWaypointV2Action> generateWaypointActions(uint16_t actionNum);
        std::vector<WaypointV2> generatePolygonWaypoints(float32_t radius,
                                                         uint16_t polygonNum);
        void getGlobalCruiseSpeed();
        void setWaypointV2Defaults(WaypointV2 &waypointV2);
        void setGlobalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed);

        // Helper Functions
        base::samples::RigidBodyState getRigidBodyState() const;
        power_base::BatteryStatus getBatteryStatus() const;
        Telemetry::Vector3f quaternionToEulerAngle(const Telemetry::Quaternion& quat);
        
        static void
        obtainJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode,
                                      UserData userData);
        static void
        releaseJoystickCtrlAuthorityCB(ErrorCode::ErrorCodeType errorCode,
                                       UserData userData);
        bool setUpSubscription(int pkgIndex, int freq,
                               Telemetry::TopicName topicList[], uint8_t topicSize);
        bool teardownSubscription(const int pkgIndex);

        void setupController();
        void setupEnvironment();
        bool initVehicle();
        bool checkTelemetrySubscription();

        static void
        startAsyncCmdCallBack(ErrorCode::ErrorCodeType retCode,
                              UserData SampleLog);

        static E_OsdkStat
        updateMissionState(T_CmdHandle *cmdHandle,
                           const T_CmdInfo *cmdInfo,
                           const uint8_t *cmdData,
                           void *userData);
        static E_OsdkStat
        updateMissionEvent(T_CmdHandle *cmdHandle,
                           const T_CmdInfo *cmdInfo,
                           const uint8_t *cmdData,
                           void *userData);
        static E_OsdkStat
        OsdkUser_Console(const uint8_t *data,
                         uint16_t dataLen);
    };
}
#endif

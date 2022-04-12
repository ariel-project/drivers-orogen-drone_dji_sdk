/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP
#define DRONE_DJI_SDK_DRONEFLIGHTCONTROLTASK_TASK_HPP

#include "djiosdk/dji_status.hpp"
#include "djiosdk/dji_telemetry.hpp"
#include "djiosdk/dji_vehicle.hpp"
#include "drone_dji_sdk/DroneFlightControlTaskBase.hpp"
#include "drone_dji_sdkTypes.hpp"
#include <gps_base/BaseTypes.hpp>
#include <gps_base/UTMConverter.hpp>

namespace drone_dji_sdk
{

    /*! \class DroneFlightControlTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine
     to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
     interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
     associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','drone_dji_sdk::DroneFlightControlTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
     argument.
     */
    class DroneFlightControlTask : public DroneFlightControlTaskBase
    {
        friend class DroneFlightControlTaskBase;

      protected:
      public:
        /** TaskContext constructor for DroneFlightControlTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        DroneFlightControlTask(
            std::string const& name = "drone_dji_sdk::DroneFlightControlTask");

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
        int64_t mFunctionTimeout = 1;
        int mStatusFreqInHz = 10;
        base::Time mPrevDataTimestamp;
        double mPositionThreshold = 1;
        Vehicle::ActivateData mActivateData;
        gps_base::UTMConverter mGPSSolution;
        std::unique_ptr<DJI::OSDK::Vehicle> mVehicle;
        drone_control::Mission mLastMission;
        DJI::OSDK::ACK::ErrorCode mAuthorityStatus;
        Status mStatus;

        bool initVehicle();
        bool missionInitSettings(drone_control::Mission wypMission);
        bool checkTelemetrySubscription();
        bool setUpSubscription(
            int pkgIndex,
            int freq,
            std::vector<Telemetry::TopicName> topicList);
        bool teardownSubscription(const int pkgIndex);

        void posControl(drone_control::VehicleSetpoint setpoint);
        void velControl(drone_control::VehicleSetpoint setpoint);
        void land(drone_control::VehicleSetpoint setpoint);
        void takeoff(drone_control::VehicleSetpoint setpoint);
        void mission(drone_control::Mission wypMission);

        States runtimeStatesTransition(DJI::OSDK::Telemetry::SDKInfo control_device);
        void applyTransition(States const& next_state);

        // Helper Functions
        bool checkDistanceThreshold(drone_control::VehicleSetpoint pos);

        gps_base::Solution convertToGPSPosition(drone_control::Waypoint cmd_waypoint);

        base::samples::RigidBodyState getRigidBodyState() const;

        power_base::BatteryStatus getBatteryStatus() const;

        WayPointInitSettings getWaypointInitDefaults(drone_control::Mission mission);

        DJI::OSDK::WayPointSettings
        getWaypointSettings(drone_control::Waypoint cmd_waypoint, int index);

        /**
         * Check if the SDK can take the drone control authority.
         *
         * @return whether the SDK can obtain control of the drone.
         */
        bool canTakeControl(DJI::OSDK::Telemetry::SDKInfo const& control_device);
    };
} // namespace drone_dji_sdk
#endif

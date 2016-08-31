/* Generated from orogen/lib/orogen/templates/task/SklearnTask.hpp */

#ifndef GP_ODOMETRY_TASK_SKLEARNTASK_HPP
#define GP_ODOMETRY_TASK_SKLEARNTASK_HPP

/** Std libraries **/
#include <list>
#include <cmath>

/** library **/
#include <gp_odometry/Sklearn.hpp>

/** SklearnTask base **/
#include "gp_odometry/SklearnTaskBase.hpp"

namespace gp_odometry {

    /*! \class SklearnTask 
     * \brief The SklearnTask context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare the Three Odometry class
    The component computes the robot pose based on
    a complete motion model.
    Robot joints positions are needed to compute
    the forward kinematics of robot chains.
    Angular and robot joints
    rates are needed to compute the movement.


    The corresponding C++ class can be edited in SklearnTasks/SklearnTask.hpp and
    SklearnTasks/SklearnTask.cpp, and will be put in the gp_odometry namespace.
     * \details
     * The name of a SklearnTaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         SklearnTask('custom_SklearnTask_name','gp_odometry::SklearnTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class SklearnTask : public SklearnTaskBase
    {
	friend class SklearnTaskBase;
    protected:

        virtual void delta_pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample);

        virtual void joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample);

        virtual void orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/
        unsigned int gp_number_samples, gp_counter_samples;
        std::vector<std::string> position_joint_names, speed_joint_names;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/
        gp_odometry::Sklearn gp_x;

        gp_odometry::Sklearn gp_y;

        gp_odometry::Sklearn gp_z;

        Eigen::Vector3d estimation;

        Eigen::Vector3d variance;

        /***************************/
        /** Input port variables  **/
        /***************************/
        ::base::Vector3d delta_position;

        std::list< ::base::Vector3d > angular_velocity_samples;

        std::list< ::base::samples::Joints > joints_samples;

        std::list< ::base::samples::RigidBodyState > orientation_samples;

        Eigen::Matrix3d cov_position;

        /***************************/
        /** Output port variables **/
        /***************************/
        ::base::samples::RigidBodyState delta_pose;


    public:
        /** SklearnTaskContext constructor for SklearnTask
         * \param name Name of the SklearnTask. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial SklearnTaskState of the SklearnTaskContext. Default is Stopped state.
         */
        SklearnTask(std::string const& name = "gp_odometry::SklearnTask");

        /** SklearnTaskContext constructor for SklearnTask 
         * \param name Name of the SklearnTask. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this SklearnTask, which serialises the execution of all commands, programs, state machines and incoming events for a SklearnTask. 
         * 
         */
        SklearnTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of SklearnTask
         */
	    ~SklearnTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the SklearnTask context definition with (for example):
         \verbatim
         SklearnTask_context "SklearnTaskName" do
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

        /**
         * */
        std::vector<double> meanSamples();

        /**
        * */
        void onlineCovariance (Eigen::Matrix3d& covariance, double x_var = 0.00,
                                double y_var = 0.00, double z_var = 0.00);

    };
}

#endif


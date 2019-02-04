/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SIMULATION_JOINTS_TASK_HPP
#define SIMULATION_JOINTS_TASK_HPP

#include "mars/JointsBase.hpp"
#include <base/commands/Joints.hpp>

namespace mars {

    /*! \class Joints
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','mars::Joints')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Joints : public JointsBase
    {
	friend class JointsBase;
    protected:
	struct JointConversion
	{
	    JointConversion()
		: mars_id(-1), scaling(1.0), offset(0.0), absolutePosition(0), lastPosition(0), gotPosition(false) {}

	    double fromMars( double v )
	    {
		return v * scaling + offset;
	    }
	    double toMars( double v )
	    {
		return (v - offset) / scaling;
	    }

            double updateAbsolutePosition( double v )
            {
                if(!gotPosition)
                {
                    gotPosition = true;
                    absolutePosition = v;
                    lastPosition = v;
                }

                double diff = v- lastPosition;

                if(diff > M_PI)
                {
                    diff -= 2* M_PI;
                }
                if(diff < -M_PI)
                {
                    diff += 2* M_PI;
                }

                absolutePosition += diff;

                lastPosition = v;

                return absolutePosition;
            }
            double getAbsolutePosition()
            {
                return absolutePosition;
            }

	    int mars_id;
            std::string marsName;
            std::string externalName;
        /// Scale factor from Mars to Module
	    double scaling;
	    double offset;
            double absolutePosition;
            double lastPosition;
            bool gotPosition;
	};
	std::vector<JointConversion> mars_ids;
	enum JointTypes{MOTOR,PASSIVE};
  std::vector<JointTypes> joint_types;
	std::vector<mars::interfaces::MotorType> motor_types;

	base::samples::Joints status;
	mars::JointCurrents currents;
	base::commands::Joints cmd;

	std::vector< mars::ParallelKinematic > parallel_kinematics;
	mars::JointPositionAndSpeedControlMode controlMode;
  bool use_torque_control;

    public:
        virtual void init();
        virtual void update(double delta_t);

        virtual bool setPosdiff2torquescale(double value);
        virtual bool setVeldiff2torquescale(double value);

        /** TaskContext constructor for Joints
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Joints(std::string const& name = "mars::Joints");

        /** TaskContext constructor for Joints
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        Joints(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Joints
         */
	~Joints();

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
    };
}

#endif

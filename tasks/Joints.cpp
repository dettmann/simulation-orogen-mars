/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars/sim/SimMotor.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <mars/interfaces/sim/ControlCenter.h>

using namespace mars;

Joints::Joints(std::string const& name)
    : JointsBase(name)
{
}

Joints::Joints(std::string const& name, RTT::ExecutionEngine* engine)
    : JointsBase(name, engine)
{
    controlMode = mars::IGNORE;
}

Joints::~Joints()
{
}

void Joints::init()
{
    // for each of the names, get the mars motor id
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
        std::string &name( mars_ids[i].marsName );
        int marsMotorId = control->motors->getID( name );
        if( marsMotorId ){
            mars_ids[i].mars_id = marsMotorId;
            joint_types.push_back(MOTOR);

            if (_torque_control.get()) {
                mars::sim::SimMotor *motor = control->motors->getSimMotor( marsMotorId );
                motor->setType(mars::interfaces::MOTOR_TYPE_VELOCITY);
            }

        }else{
            int marsJointId = control->joints->getID( name );
            if (marsJointId){
                mars_ids[i].mars_id = marsJointId;
                joint_types.push_back(PASSIVE);
            }else{
                throw std::runtime_error("there is no motor or joint by the name of " + name);
            }
        }

    }
}

void Joints::update(double delta_t)
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
    // if there was a command, write it to the mars
    while( _command.read( cmd ) == RTT::NewData )
    {
	for( size_t i=0; i<mars_ids.size(); ++i )
	{
        //passive joints can't take commands
	    if (joint_types [i] == PASSIVE){
	        continue;
	    }

	    // for each command input look up the name in the mars_ids structure
	    JointConversion conv = mars_ids[i];

            //ignore the case that the input data stream has not commands for our other joints
            std::vector<std::string>::const_iterator it = std::find(cmd.names.begin(), cmd.names.end(), conv.externalName);
            if (it == cmd.names.end()){
                continue;
            }

            base::JointState &curCmd(cmd[*it]);

	    mars::sim::SimMotor *motor = control->motors->getSimMotor( conv.mars_id );

      if (!_torque_control.get())
      {

      if( curCmd.hasPosition() )
            {
                //set maximum speed that is allowed for turning

                if(curCmd.hasSpeed()){
                    switch (controlMode){
                    case IGNORE:break;
                    case MAX_SPEED:motor->setMaxSpeed(curCmd.speed);break;
                    //case SPEED_AT_POS: RTT::log(RTT::Error) << "SPEED_AT_POS" << RTT::endlog();break
                    }
	            }
                motor->setControlValue( conv.toMars( curCmd.position ) );
            }
            else
            {
                if( curCmd.hasSpeed() )
                    motor->setVelocity(curCmd.speed / conv.scaling);
            }

            if( curCmd.hasEffort() )
            {
                LOG_WARN_S << "Effort command ignored";
            }

    } else {
      // torque control is realized via limiting maximum torque and setting a high speed reference
      motor->setMaxEffort(curCmd.effort);
      double threshold = 0.001;
      if (curCmd.speed > threshold) {
          motor->setControlValue(1000.0);
      } else if (curCmd.speed < -threshold) {
          motor->setControlValue(-1000.0);
      } else {
          motor->setControlValue(0.0);
      }


    }
	    if( curCmd.hasRaw() )
	    {
               LOG_WARN_S << "Raw command ignored";
	    }
	}
    }

    // in any case read out the status
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
        JointConversion *conv = NULL;

        if (parallel_kinematics.empty()){
            conv = &(mars_ids[i]);
        }else{
            //mars_id does not fit the index of status,
            //find the conv by status name
            for (std::vector<JointConversion>::iterator it = mars_ids.begin();it != mars_ids.end();it++){
                if (it->externalName == status.names[i]){
                    conv = &(*it);
                    break;
                }
            }

        }

        base::JointState state;

        if (joint_types[i] == MOTOR){
            mars::sim::SimMotor *motor = control->motors->getSimMotor( conv->mars_id );

            state.position = conv->fromMars(conv->updateAbsolutePosition( motor->getActualPosition() ));
            state.speed = motor->getJoint()->getVelocity() * conv->scaling;
            state.effort = conv->fromMars( motor->getTorque() );

            currents[conv->externalName] = motor->getCurrent();

            status[conv->externalName] = state;
        }else{
            mars::sim::SimJoint* joint = control->joints->getSimJoint( conv->mars_id );

            state.position = conv->fromMars(conv->updateAbsolutePosition( joint->getActualAngle1() ));
            state.speed = joint->getVelocity() * conv->scaling;
            state.effort = 0;

            currents[conv->externalName] = 0;
            status[conv->externalName] = state;

        }
    }

    // and write it to the output port
    status.time = getTime();
    _status_samples.write( status );

    currents.time = status.time;
    _current_values.write(currents);

    // see if we have configuration for the joint_transforms
    // and the output port for it is connected
    std::vector<base::samples::RigidBodyState> rbs;
    if( !_joint_transform.value().empty() && _transforms.connected() )
    {
        _joint_transform.value().setRigidBodyStates( status, rbs );
        for( size_t i=0; i < rbs.size(); ++i )
            _transforms.write( rbs[i] );
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Joints.hpp for more detailed
// documentation about them.

bool Joints::configureHook()
{
    size_t num_joints = _names.value().size();

    // test if scaling is valid
    if( !_scaling.value().empty() && _scaling.value().size() != num_joints )
    {
	LOG_ERROR_S << "The scaling property needs to be empty or of the same size as names.";
	return false;
    }
    if( !_offset.value().empty() && _offset.value().size() != num_joints )
    {
	LOG_ERROR_S << "The offset property needs to be empty or of the same size as names.";
	return false;
    }

    std::vector< mars::ParallelKinematic > parallel_kinematics = _parallel_kinematics.value();


    mars_ids.clear();
    // fill the joint structure
    mars_ids.resize( num_joints );
    cmd.resize( num_joints);
    status.resize( num_joints - parallel_kinematics.size() );


    currents.resize(num_joints - parallel_kinematics.size());


    std::vector<std::string> marsNames = _names.value();



    //set proper status names (by parallel kinematics)
    //status.resize( num_joints - parallel_kinematics.size());

    //set names
    if (parallel_kinematics.empty()){
    	status.names = _names.value();
    	currents.names = _names.value();
    }else{
    	printf("parallel kinematic_configuration:\n");
	    for (std::vector< mars::ParallelKinematic >::iterator it = parallel_kinematics.begin();it != parallel_kinematics.end();it++){
	    	printf("%s -> %s, %s\n",it->externalName.c_str(),it->internalName1.c_str(),it->internalName2.c_str());
	    }
	    std::vector<std::string> externalNames;
	    //set names for status
	    bool is_parallel = false;
	    for (std::vector<std::string>::iterator name = marsNames.begin(); name != marsNames.end();name++){
	    	is_parallel = false;
	    	for (std::vector< mars::ParallelKinematic >::iterator parallel = parallel_kinematics.begin();parallel != parallel_kinematics.end();parallel++){
	    		if (*name == parallel->internalName1){
	    			externalNames.push_back(parallel->externalName);
	    			is_parallel = true;
	    		}else if(*name == parallel->internalName2){
	    			//second is NOT used for status data
	    			is_parallel = true;
	    		}
	    	}
	    	if (!is_parallel){
	    		externalNames.push_back(*name);
	    	}
	    }
	    status.names = externalNames;
	    currents.names = externalNames;

    }


    std::vector<std::string> rename = _name_remap.get();
    for( size_t i=0; i<mars_ids.size(); i++ )
    {
		if( !_scaling.value().empty() ){
			mars_ids[i].scaling = _scaling.value()[i];
		}

		if( !_offset.value().empty() ){
			mars_ids[i].offset = _offset.value()[i];
		}
        mars_ids[i].marsName = marsNames[i];

		if(rename.empty() || rename[i].empty())
		{
			mars_ids[i].externalName = marsNames[i];

		}else
		{
			status.names[i] = rename[i];
            currents.names[i] = rename[i];
			mars_ids[i].externalName = rename[i];
		}

        if (!parallel_kinematics.empty()){
    	    //if (marsNames[i] in _parallel_kinematics )
    	    for (std::vector< mars::ParallelKinematic >::iterator it = parallel_kinematics.begin();it != parallel_kinematics.end();it++){
    	    	if (mars_ids[i].marsName == it->internalName1 || mars_ids[i].marsName == it->internalName2){
    	    		mars_ids[i].externalName = it->externalName;
    	    	}
    	    }
        }

    }

    controlMode = _controlMode.value();


    //this needs to be called here, or we get a race condition
    //between the init of the plugin and the filling of mars_ids
    return JointsBase::configureHook();
}
bool Joints::startHook()
{
    if (! JointsBase::startHook())
        return false;
    return true;
}
void Joints::updateHook()
{
    JointsBase::updateHook();
}
void Joints::errorHook()
{
    JointsBase::errorHook();
}
void Joints::stopHook()
{
    JointsBase::stopHook();
}
void Joints::cleanupHook()
{
    JointsBase::cleanupHook();
}

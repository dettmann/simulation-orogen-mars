/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "RobotTeleportation.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
//#include <mars/interfaces/sim/LoadCenter.h>
//#include <mars/interfaces/sim/LoadSceneInterface.h>
//#include <mars/smurf_loader/SMURFLoader.h>
#include <mars/sim/SimEntity.h>

#include <base-logging/Logging.hpp>


namespace mars {

  using namespace interfaces;
  using namespace utils;


  RobotTeleportation::RobotTeleportation(std::string const& name)
      : RobotTeleportationBase(name)
  {
  }

  RobotTeleportation::RobotTeleportation(std::string const& name, RTT::ExecutionEngine* engine)
      : RobotTeleportationBase(name, engine)
  {
  }

  RobotTeleportation::~RobotTeleportation()
  {
  }

  void RobotTeleportation::init()
  {
  }

  void RobotTeleportation::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See RobotTeleportation.hpp for more detailed
  // documentation about them.

  bool RobotTeleportation::configureHook()
  {
    if (! RobotTeleportationBase::configureHook()) return false;
    //get robot data
    robot_name = _robot_name.get();
    robot_entity = control->entities->getEntity(robot_name);
    LOG_DEBUG_S << "RobotTeleportation: " << "got robot: "<< robot_name;
    reset_node_name = _reset_node_name.get();
    LOG_DEBUG_S << "RobotTeleportation: " << "got robot: "<< reset_node_name;
    //get config properties
    pos_mode = _position_mode.get();
    LOG_DEBUG_S << "RobotTeleportation: " << "got config";
    //save initial position
    target.id = 0;
    target.cfg = robot_entity->getConfig();
    //initiate target vector
    targets.push_back(target);
    LOG_DEBUG_S << "RobotTeleportation: " << "got current state";
    //get target positions from file
    if (pos_mode == 1) {
      //load target config list
      ConfigVector::iterator mIt = target.cfg["teleportation_targets"].begin();
      unsigned int id = 1; //first item is initial position
      for (; mIt != target.cfg["teleportation_targets"].end(); ++mIt) {
        Target t;
        t.id = ++id;
        t.cfg = target.cfg;
        for (FIFOMap<std::string, ConfigItem>::iterator it = mIt->beginMap(); it!=mIt->endMap(); ++it) {
          if (it->first == "file") {
            unsigned int pos = ((std::string)it->second).find_last_of("/")+1;
            if (pos != std::string::npos) {
              t.cfg["file"] = ((std::string)it->second).substr(pos);
              t.cfg["path"] = (std::string)target.cfg["load_path"]+((std::string)it->second).substr(0,pos);
            } else {
              t.cfg[it->first] = it->second;
            }
          } else {
            t.cfg[it->first] = it->second;
          }
        }
        targets.push_back(t);
      }
      LOG_DEBUG_S << "RobotTeleportation: " << "read targets";
    }

    LOG_DEBUG_S << "RobotTeleportation: "<< "Task configured! " << (pos_mode ? "(Manual)":"(Preconfigured)");

    return true;
  }

  bool RobotTeleportation::startHook()
  {
    return RobotTeleportationBase::startHook();
  }

  void RobotTeleportation::updateHook()
  {
    RobotTeleportationBase::updateHook();
    bool reloadScene = false;

    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    //if triggered load new position
    if (_position_id.readNewest(curr_id) == RTT::NewData ||
        _position.readNewest(pos) == RTT::NewData ||
        _rotation.readNewest(rot) == RTT::NewData ||
        _anchor.readNewest(anchor) == RTT::NewData ||
        _reset_node.readNewest(reset_node) == RTT::NewData
       )
    {
      LOG_DEBUG_S << "RobotTeleportation: " << "updateHook triggered!";
      if (pos_mode == 1){
        if (curr_id >=targets.size()) {
          LOG_DEBUG_S << "RobotTeleportation: " << "given to high id " << curr_id << ">=" <<targets.size()<<"!";
          curr_id = 0;
        }
        if (targets[curr_id].cfg.hasKey("file") && (target.cfg["file"] != targets[curr_id].cfg["file"] || target.cfg["path"] != targets[curr_id].cfg["path"])) {
          reloadScene = true;
        }
        target = targets[curr_id];
      } else {
        target.id = -1;
        target.cfg["position"][0] = pos[0];
        target.cfg["position"][1] = pos[1];
        target.cfg["position"][2] = pos[2];
        sRotation rot_e = quaternionTosRotation(rot);
        target.cfg["rotation"][0] = rot_e.alpha;
        target.cfg["rotation"][1] = rot_e.beta;
        target.cfg["rotation"][2] = rot_e.gamma;
        if (anchor) {
          target.cfg["anchor"] = "world";
        } else {
          target.cfg["anchor"] = "none";
        }
        target.cfg["reset_node"] = reset_node;
      }

      if (reloadScene) {
        LOG_DEBUG_S << "RobotTeleportation: Reloading the following cfg:\n" << target.cfg.toYamlString();
        control->entities->removeEntity(robot_name);
        control->sim->loadScene((std::string)target.cfg["path"]+(std::string)target.cfg["file"], robot_name);
        robot_entity = control->entities->getEntity(robot_name);
      }

      robot_entity->setInitialPose(robot_entity->hasAnchorJoint(), &target.cfg);

      /* if the robot has suspension joints we have to teleport the root node to
       the new position as well*/
      NodeId id = control->nodes->getID(reset_node_name);
      if (id == INVALID_ID) LOG_DEBUG_S << "RobotTeleportation: "<< "Reset node not found: "<< reset_node_name <<"!";
      if (target.cfg.find("reset_node") != target.cfg.end() && target.cfg["reset_node"] && id != INVALID_ID) {
        LOG_DEBUG_S << "RobotTeleportation: "<< "Resetting node "<< reset_node_name <<"!";
        Quaternion tmpQ(1, 0, 0, 0);
        Vector tmpV;
        NodeData rootNode = control->nodes->getFullNode(id);
        if(target.cfg.find("position") != target.cfg.end()) {
          rootNode.pos.x() = target.cfg["position"][0];
          rootNode.pos.y() = target.cfg["position"][1];
          rootNode.pos.z() = target.cfg["position"][2];
          control->nodes->editNode(&rootNode, EDIT_NODE_POS | EDIT_NODE_MOVE_ALL);
        }
        if(target.cfg.find("rotation") != target.cfg.end()) {
          // check if euler angles or quaternion is provided; rotate around z
          // if only one angle is provided
          switch (target.cfg["rotation"].size()) {
          case 1: tmpV[0] = 0;
            tmpV[1] = 0;
            tmpV[2] = target.cfg["rotation"][0];
            tmpQ = eulerToQuaternion(tmpV);
            break;
          case 3: tmpV[0] = target.cfg["rotation"][0];
            tmpV[1] = target.cfg["rotation"][1];
            tmpV[2] = target.cfg["rotation"][2];
            tmpQ = utils::eulerToQuaternion(tmpV);
            break;
          case 4: tmpQ.x() = (sReal)target.cfg["rotation"][1];
            tmpQ.y() = (sReal)target.cfg["rotation"][2];
            tmpQ.z() = (sReal)target.cfg["rotation"][3];
            tmpQ.w() = (sReal)target.cfg["rotation"][0];
            break;
          }
          rootNode.rot = tmpQ;
          control->nodes->editNode(&rootNode, EDIT_NODE_ROT | EDIT_NODE_MOVE_ALL);
        }
      }

      LOG_DEBUG_S << "RobotTeleportation: "<< "Robot teleportation done!";
    }
  }

  // void RobotTeleportation::errorHook()
  // {
  //     RobotTeleportationBase::errorHook();
  // }

  void RobotTeleportation::stopHook()
  {
    RobotTeleportationBase::stopHook();
  }

  // void RobotTeleportation::cleanupHook()
  // {
  //     RobotTeleportationBase::cleanupHook();
  // }
}


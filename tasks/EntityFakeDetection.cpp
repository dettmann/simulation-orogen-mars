/* Generated from orogen/lib/orogen/templates/tasks/IMU.cpp */

#include "EntityFakeDetection.hpp"
#include "Plugin.hpp"
#include <mars/interfaces/sim/EntityManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/sim/SimEntity.h>

#include <base-logging/Logging.hpp>

namespace mars {

  using namespace interfaces;


  EntityFakeDetection::EntityFakeDetection(std::string const& name)
      : EntityFakeDetectionBase(name)
  {
  }

  EntityFakeDetection::EntityFakeDetection(std::string const& name, RTT::ExecutionEngine* engine)
      : EntityFakeDetectionBase(name, engine)
  {
  }

  EntityFakeDetection::~EntityFakeDetection()
  {
  }

  void EntityFakeDetection::init()
  {
  }

  void EntityFakeDetection::update(double delta_t)
  {
  }

  /// The following lines are template definitions for the various state machine
  // hooks defined by Orocos::RTT. See EntityFakeDetection.hpp for more detailed
  // documentation about them.

  bool EntityFakeDetection::configureHook()
  {
    if (! EntityFakeDetectionBase::configureHook())
      return false;

    frame_id = FrameId(_frame_id.get());
    minVisibleVertices = _minVisibleVertices.get();
    use_camera = (bool) _use_camera.get();

    if (use_camera == false) {
      frame_id = FrameId::GLOBAL;
    }

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task configured! " << (use_camera ? "(Using Camera)":"(Not Using Camera)");

    return true;
  }

  bool EntityFakeDetection::startHook()
  {
    if (use_camera == true) {
      if (!EntityFakeDetectionBase::startHook()) {
        if (!camera) {
          LOG_ERROR_S << "EntityFakeDetection"<< "Start of CameraPlugin failed! Using without camera!";
          use_camera = false;
        }
      }
    }

    //get all entities
    visible_entities = *(control->entities->subscribeToEntityCreation(nullptr));
    detectionArray = new Detection3DArray(visible_entities.size());

    LOG_DEBUG_S << "EntityFakeDetection"<< "Task started!";

    return true;
  }

  void EntityFakeDetection::updateHook()
  {
      EntityFakeDetectionBase::updateHook();

      if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning
      LOG_DEBUG_S << "EntityFakeDetection"<< "Running updateHook!";

      if (use_camera == true) { // then fill the buffer only with the entities seen by the camera
        camera->getEntitiesInView(visible_entities, minVisibleVertices);
        if (visible_entities.size()!=detectionArray->detections.size()) {
          delete detectionArray;
          detectionArray = new Detection3DArray(visible_entities.size());
        }
      } else {
        visible_entities = *(control->entities->subscribeToEntityCreation(nullptr));
        frame_id = FrameId::GLOBAL;
        minVisibleVertices = 0;
      }

      LOG_DEBUG_S<<"EntityFakeDetection"<< "got " << visible_entities.size() << " entities updateHook()";
      //set general header
      detectionArray->header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
      detectionArray->header.seq = seq++;
      detectionArray->header.frame_id = (int) frame_id;
      //generate detections
      utils::Vector center;
      utils::Quaternion rotation;
      unsigned int i = 0;
      for (std::map<unsigned long, sim::SimEntity*>::iterator iter = visible_entities.begin();
          iter != visible_entities.end(); ++iter) {
        LOG_DEBUG_S << "detecting";
        //Header
        detectionArray->detections[i].header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].header.seq = seq++;
        detectionArray->detections[i].header.frame_id = (int) frame_id;
        //BoundingBox3D
        iter->second->getBoundingBox(center,
                                     rotation,
                                     detectionArray->detections[i].bbox.size);
        detectionArray->detections[i].bbox.center.position = center;
        detectionArray->detections[i].bbox.center.orientation = rotation;
        //ObjectHypothesisWithPose
        detectionArray->detections[i].results[0].id = iter->first;
        detectionArray->detections[i].results[0].type = iter->second->getName();
        detectionArray->detections[i].results[0].pose.pose.position = center;
        detectionArray->detections[i].results[0].pose.pose.orientation = rotation;
        //PointCloud
        detectionArray->detections[i].source_cloud.header.stamp = base::Time::fromMilliseconds(control->sim->getTime());
        detectionArray->detections[i].source_cloud.header.seq = seq++;
        detectionArray->detections[i].source_cloud.header.frame_id = (int) frame_id;
        //TODO detectionArray->detections[i].source_cloud.width = control->nodes->getFullNode(rootId).mesh.vertexcount;
        //detectionArray->detections[i].source_cloud.points = control->nodes->getFullNode(rootId).mesh.vertices;// REVIEW+
        i++;
      }

      //write to rock outputs
      _detectionArray.write(*detectionArray);
  }
  // void EntityFakeDetection::errorHook()
  // {
  //     EntityFakeDetectionBase::errorHook();
  // }
  void EntityFakeDetection::stopHook()
  {
      if (use_camera == true) {
        EntityFakeDetectionBase::stopHook();
      }
      mars::Plugin::stopHook();

  }
  // void EntityFakeDetection::cleanupHook()
  // {
  //     EntityFakeDetectionBase::cleanupHook();
  // }

  void EntityFakeDetection::getData()
  {
  }
}


/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: PtuPose.cc
 +Description: Implements a Gazebo plugin to control the pose of a pan-tilt unit (PTU).
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/
#include "mapping_gazebo/PtuPose.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose.pb.h>        

#include <string>
#include <unordered_set>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

using namespace gz;
using namespace sim;
using namespace systems;

gz::math::Pose3d pose = gz::math::Pose3d::Zero;
gz::math::Pose3d pose_ant = gz::math::Pose3d::Zero;

class gz::sim::systems::PtuPosePrivate
{

  public: void OnCmdPos(const msgs::Double &_msg);


  public: void SubsHandler(const msgs::Pose &_msg);

  public: transport::Node node;

  public: std::vector<Entity> jointEntities;

  public: std::vector<std::string> jointNames;

  public: double jointPosCmd{0.0};

  public: int actuatorNumber = 0;

  public: std::mutex jointCmdMutex;


  public: bool isMaxSet {false};


  public: Model model{kNullEntity};


  public: bool useActuatorMsg{false};


  public: math::PID posPid;


  public: unsigned int jointIndex = 0u;


  enum OperationMode
  {

    PID,

    ABS
  };


  public: OperationMode mode = OperationMode::PID;

};

//////////////////////////////////////////////////
PtuPose::PtuPose()
  : dataPtr(std::make_unique<PtuPosePrivate>())
{
}

//////////////////////////////////////////////////
void PtuPose::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "PtuPose plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }





  // Subscribe to commands
  std::string topic;

  topic = transport::TopicUtils::AsValidTopic("set_ptu_position");
  this->dataPtr->node.Subscribe(topic, &PtuPosePrivate::SubsHandler, this->dataPtr.get());
  gzmsg << "PtuPose subscribing to Position messages on [" << topic << "]" << std::endl;

}

//////////////////////////////////////////////////
void PtuPose::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    
    if (pose_ant!=pose){
        //math::Pose3d projectilePose = this->dataPtr->pose;

        //gz::sim::Model::SetWorldPoseCmd(gz::sim::systems::PtuPosePrivate->dataPtr->model.Entity());
        this->dataPtr->model.SetWorldPoseCmd(_ecm, pose);
        pose_ant=pose;
    }
}
//////////////////////////////////////////////////
void PtuPosePrivate::SubsHandler(const msgs::Pose &_msg)
{
  pose = gz::msgs::Convert(_msg);
  
}


GZ_ADD_PLUGIN(PtuPose,
                    System,
                    PtuPose::ISystemConfigure,
                    PtuPose::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(PtuPose,
                          "gz::sim::systems::PtuPose")
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
  /// \brief Callback for position subscription
  /// \param[in] _msg Position message
  public: void OnCmdPos(const msgs::Double &_msg);

  /// \brief Callback for actuator position subscription
  /// \param[in] _msg Position message
  public: void SubsHandler(const msgs::Pose &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Joint name
  public: std::vector<std::string> jointNames;

  /// \brief Commanded joint position
  public: double jointPosCmd{0.0};

  /// \brief Index of position actuator.
  public: int actuatorNumber = 0;

  /// \brief mutex to protect joint commands
  public: std::mutex jointCmdMutex;

  /// \brief Is the maximum PID gain set.
  public: bool isMaxSet {false};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True if using Actuator msg to control joint position.
  public: bool useActuatorMsg{false};

  /// \brief Position PID controller.
  public: math::PID posPid;

  /// \brief Joint index to be used.
  public: unsigned int jointIndex = 0u;

  /// \brief Operation modes
  enum OperationMode
  {
    /// \brief Use PID to achieve positional control
    PID,
    /// \brief Bypass PID completely. This means the joint will move to that
    /// position bypassing the physics engine.
    ABS
  };

  /// \brief Joint position mode
  public: OperationMode mode = OperationMode::PID;

  //public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;
  //public: gz::math::Pose3d pose_ant = gz::math::Pose3d::Zero;
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

//   // Get params from SDF
//   auto sdfElem = _sdf->FindElement("joint_name");
//   while (sdfElem)
//   {
//     if (!sdfElem->Get<std::string>().empty())
//     {
//       this->dataPtr->jointNames.push_back(sdfElem->Get<std::string>());
//     }
//     else
//     {
//       gzerr << "<joint_name> provided but is empty." << std::endl;
//     }
//     sdfElem = sdfElem->GetNextElement("joint_name");
//   }
//   if (this->dataPtr->jointNames.empty())
//   {
//     gzerr << "Failed to get any <joint_name>." << std::endl;
//     return;
//   }

//   if (_sdf->HasElement("joint_index"))
//   {
//     this->dataPtr->jointIndex = _sdf->Get<unsigned int>("joint_index");
//   }



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
//   GZ_PROFILE("PtuPose::PreUpdate");

//   // \TODO(anyone) This is a temporary fix for
//   // gazebosim/gz-sim#2165 until gazebosim/gz-sim#2217 is resolved.
//   if (kNullEntity == this->dataPtr->model.Entity())
//   {
//     return;
//   }

//   if (!this->dataPtr->model.Valid(_ecm))
//   {
//     gzwarn << "PtuPose model no longer valid. "
//            << "Disabling plugin." << std::endl;
//     this->dataPtr->model = Model(kNullEntity);
//     return;
//   }

//   // \TODO(anyone) Support rewind
//   if (_info.dt < std::chrono::steady_clock::duration::zero())
//   {
//     gzwarn << "Detected jump back in time ["
//         << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
//         << "s]. System may not work properly." << std::endl;
//   }

//   // If the joints haven't been identified yet, look for them
//   if (this->dataPtr->jointEntities.empty())
//   {
//     bool warned{false};
//     for (const std::string &name : this->dataPtr->jointNames)
//     {
//       // First try to resolve by scoped name.
//       Entity joint = kNullEntity;
//       auto entities = entitiesFromScopedName(
//           name, _ecm, this->dataPtr->model.Entity());

//       if (!entities.empty())
//       {
//         if (entities.size() > 1)
//         {
//           gzwarn << "Multiple joint entities with name ["
//                 << name << "] found. "
//                 << "Using the first one.\n";
//         }
//         joint = *entities.begin();

//         // Validate
//         if (!_ecm.EntityHasComponentType(joint, components::Joint::typeId))
//         {
//           gzerr << "Entity with name[" << name
//                 << "] is not a joint\n";
//           joint = kNullEntity;
//         }
//         else
//         {
//           gzdbg << "Identified joint [" << name
//                 << "] as Entity [" << joint << "]\n";
//         }
//       }

//       if (joint != kNullEntity)
//       {
//         this->dataPtr->jointEntities.push_back(joint);
//       }
//       else if (!warned)
//       {
//         gzwarn << "Failed to find joint [" << name << "]\n";
//         warned = true;
//       }
//     }
//   }
//   if (this->dataPtr->jointEntities.empty())
//     return;

//   // Nothing left to do if paused.
//   if (_info.paused)
//     return;

//   // Create joint position component if one doesn't exist
//   auto jointPosComp = _ecm.Component<components::JointPosition>(
//       this->dataPtr->jointEntities[0]);
//   if (!jointPosComp)
//   {
//     _ecm.CreateComponent(this->dataPtr->jointEntities[0],
//         components::JointPosition());
//   }

//   // We just created the joint position component, give one iteration for the
//   // physics system to update its size
//   if (jointPosComp == nullptr || jointPosComp->Data().empty())
//     return;

//   // Sanity check: Make sure that the joint index is valid.
//   if (this->dataPtr->jointIndex >= jointPosComp->Data().size())
//   {
//     static std::unordered_set<Entity> reported;
//     if (reported.find(this->dataPtr->jointEntities[0]) == reported.end())
//     {
//       gzerr << "[PtuPose]: Detected an invalid <joint_index> "
//              << "parameter. The index specified is ["
//              << this->dataPtr->jointIndex << "] but joint ["
//              << this->dataPtr->jointNames[0] << "] only has ["
//              << jointPosComp->Data().size() << "] index[es]. "
//              << "This controller will be ignored" << std::endl;
//       reported.insert(this->dataPtr->jointEntities[0]);
//     }
//     return;
//   }

//   // Get error in position
//   double error;
//   {
//     std::lock_guard<std::mutex> lock(this->dataPtr->jointCmdMutex);
//     error = jointPosComp->Data().at(this->dataPtr->jointIndex) -
//             this->dataPtr->jointPosCmd;
//   }

//   // Check if the mode is ABS
//   if (this->dataPtr->mode ==
//     PtuPosePrivate::OperationMode::ABS)
//   {
//     // Calculate target velcity
//     double targetVel = 0;

//     // Get time in seconds
//     auto dt = std::chrono::duration<double>(_info.dt).count();

//     // Get the maximum amount in m that this joint may move
//     auto maxMovement = this->dataPtr->posPid.CmdMax() * dt;

//     // Limit the maximum change to maxMovement
//     if (abs(error) > maxMovement && this->dataPtr->isMaxSet)
//     {
//       targetVel = (error < 0) ? this->dataPtr->posPid.CmdMax() :
//         -this->dataPtr->posPid.CmdMax();
//     }
//     else
//     {
//       targetVel = - error / dt;
//     }
//     for (Entity joint : this->dataPtr->jointEntities)
//     {
//       // Update velocity command.
//       auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

//       if (vel == nullptr)
//       {
//         _ecm.CreateComponent(
//             joint, components::JointVelocityCmd({targetVel}));
//       }
//       else
//       {
//         *vel = components::JointVelocityCmd({targetVel});
//       }
//     }
//     return;
//   }

//   for (Entity joint : this->dataPtr->jointEntities)
//   {
//     // Update force command.
//     double force = this->dataPtr->posPid.Update(error, _info.dt);

//     auto forceComp =
//         _ecm.Component<components::JointForceCmd>(joint);
//     if (forceComp == nullptr)
//     {
//       _ecm.CreateComponent(joint,
//                           components::JointForceCmd({force}));
//     }
//     else
//     {
//       *forceComp = components::JointForceCmd({force});
//     }
//   }
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
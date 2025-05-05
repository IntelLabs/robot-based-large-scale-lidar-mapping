/**
Copyright (C) 2025 Intel Corporation
SPDX-License-Identifier: Apache-2.0
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
 +File name: PtuCollision.cc
 +Description: Implements a Gazebo plugin to detect and handle collisions for a pan-tilt unit (PTU).
 +Author: Javier Felix-Rendon
 +Mail: javier.felix.rendon@intel.com
 +Version: 1.0
 +Date: 5/2/2025
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
*/
#include "mapping_gazebo/PtuCollision.hh"


#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PtuCollisionPrivate
{
 
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);


  public: void Enable(const bool _value);


  public: void Update(const UpdateInfo &_info,
                      const EntityComponentManager &_ecm);


  public: void AddTargetEntities(const EntityComponentManager &_ecm,
                                 const std::vector<Entity> &_entities);

  
  public: Model model{kNullEntity};


  transport::Node node;


  public: std::vector<Entity> collisionEntities;


  public: std::string targetName;

  public: std::vector<Entity> targetEntities;

  public: using DurationType = std::chrono::duration<double>;


  public: DurationType targetTime{0};


  public: DurationType touchStart{0};

  public: std::string ns;

 
  public: std::optional<transport::Node::Publisher> touchedPub;


  public: sdf::ElementPtr sdfConfig;

 
  public: bool initialized{false};

  public: bool validConfig{false};

  public: bool enabled{false};

  public: bool enableInitialValue{false};


  public: std::mutex serviceMutex;
};

//////////////////////////////////////////////////
void PtuCollision::Reset(const gz::sim::UpdateInfo &/*_info*/,
  gz::sim::EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->Enable(this->dataPtr->enableInitialValue);
}

//////////////////////////////////////////////////
void PtuCollisionPrivate::Load(const EntityComponentManager &_ecm,
                         const sdf::ElementPtr &_sdf)
{

  if (!_sdf->HasElement("target"))
  {
    gzerr << "Missing required parameter <target>." << std::endl;
    return;
  }

  this->targetName = _sdf->GetElement("target")->Get<std::string>();

  std::vector<Entity> potentialEntities;
  _ecm.Each<components::Collision>(
      [&](const Entity &_entity, const components::Collision *) -> bool
      {
        potentialEntities.push_back(_entity);
        return true;
      });

  this->AddTargetEntities(_ecm, potentialEntities);


  auto allLinks =
      _ecm.ChildrenByComponents(this->model.Entity(), components::Link());

  for (const Entity linkEntity : allLinks)
  {
    auto linkCollisions =
        _ecm.ChildrenByComponents(linkEntity, components::Collision());
    
    //auto bbox = linkEntityCollisionBoundingBox();

    for (const Entity colEntity : linkCollisions)
    {
      if (_ecm.EntityHasComponentType(colEntity,
                                      components::ContactSensorData::typeId))
      {
        this->collisionEntities.push_back(colEntity);
      }
    }
  }

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }
  this->ns = transport::TopicUtils::AsValidTopic(_sdf->Get<std::string>(
      "namespace"));
  if (this->ns.empty())
  {
    gzerr << "<namespace> [" << _sdf->Get<std::string>("namespace")
           << "] is invalid." << std::endl;
    return;
  }

  // Target time
  if (!_sdf->HasElement("time"))
  {
    gzerr << "Missing required parameter <time>" << std::endl;
    return;
  }

  this->targetTime = DurationType(_sdf->Get<double>("time"));

  // Start/stop "service"
  std::string enableService{"/" + this->ns + "/enable"};
  std::function<void(const msgs::Boolean &)> enableCb =
      [this](const msgs::Boolean &_req)
      {
        this->Enable(_req.data());
      };
  this->node.Advertise(enableService, enableCb);

  this->validConfig = true;

  // Start enabled or not
  if (_sdf->Get<bool>("enabled", false).first)
  {
    this->enableInitialValue = true;
    this->Enable(true);
  }
}

//////////////////////////////////////////////////
void PtuCollisionPrivate::Enable(const bool _value)
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);

  if (_value)
  {
    if (!this->touchedPub.has_value()){
      this->touchedPub = this->node.Advertise<msgs::Boolean>(
          "/" + this->ns);
    }

    this->touchStart = DurationType::zero();
    this->enabled = true;

    gzdbg << "Started touch plugin [" << this->ns << "]" << std::endl;
  }
  else
  {
    this->enabled = false;

    gzdbg << "Stopped touch plugin [" << this->ns << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
PtuCollision::PtuCollision()
    : System(), dataPtr(std::make_unique<PtuCollisionPrivate>())
{
}

//////////////////////////////////////////////////
void PtuCollisionPrivate::Update(const UpdateInfo &_info,
                                const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PtuCollisionPrivate::Update");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (!this->enabled)
      return;
  }

  if (_info.paused)
    return;

  for (const Entity colEntity : this->collisionEntities)
  {
    auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);
    msgs::Boolean msg;
    msg.set_data(false);  //No collision
    
    if (contacts)
    {


      // Check if the contacts include one of the target entities.
      for (const auto &contact : contacts->Data().contact())
      {
        bool col1Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision1().id());
        bool col2Target = std::binary_search(this->targetEntities.begin(),
            this->targetEntities.end(),
            contact.collision2().id());
        if (col1Target || col2Target)
        {

          msg.set_data(true); //Collision
        }

      }
    }

    this->touchedPub->Publish(msg); // Publish message

  }
}

//////////////////////////////////////////////////
void PtuCollisionPrivate::AddTargetEntities(const EntityComponentManager &_ecm,
                                           const std::vector<Entity> &_entities)
{
  if (_entities.empty())
    return;

  for (Entity entity : _entities)
  {

    std::string name = scopedName(entity, _ecm);
    if (name.find(this->targetName) != std::string::npos)
    {
      this->targetEntities.push_back(entity);
    }
  }

  // Sort so that we can do binary search later on.
  std::sort(this->targetEntities.begin(), this->targetEntities.end());
}

//////////////////////////////////////////////////
void PtuCollision::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Touch plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void PtuCollision::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("PtuCollision::PreUpdate");
  if ((!this->dataPtr->initialized) && this->dataPtr->sdfConfig)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;
  }

  // If Load() was successful, validConfig is set to true
  if (this->dataPtr->validConfig)
  {
    // Update target entities when new collisions are added
    std::vector<Entity> potentialEntities;
    _ecm.EachNew<components::Collision>(
        [&](const Entity &_entity, const components::Collision *) -> bool
        {
          potentialEntities.push_back(_entity);
          return true;
        });
    this->dataPtr->AddTargetEntities(_ecm, potentialEntities);
  }
}

//////////////////////////////////////////////////
void PtuCollision::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PtuCollision::PostUpdate");
  if (this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_info, _ecm);
  }
}

GZ_ADD_PLUGIN(PtuCollision,
              System,
              PtuCollision::ISystemConfigure,
              PtuCollision::ISystemPreUpdate,
              PtuCollision::ISystemPostUpdate,
              PtuCollision::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(PtuCollision, "gz::sim::systems::PtuCollision")
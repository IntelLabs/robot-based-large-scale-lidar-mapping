// /*
//  * Copyright (C) 2022 Open Source Robotics Foundation
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
// */

// // We'll use a string and the gzmsg command below for a brief example.
// // Remove these includes if your plugin doesn't need them.
// #include <string>
// #include <gz/common/Console.hh>
// #include <gz/sim/components/JointPosition.hh>
// #include <gz/math/Helpers.hh>
// #include <gz/math/Pose3.hh>
// #include <gz/math/PID.hh>
// #include <gz/math/Vector3.hh>
// #include <gz/sim/components/Name.hh>
// #include <gz/sim/components/Pose.hh>
// #include <gz/transport/Node.hh>
// #include <sdf/sdf.hh>
// #include <gz/sim/Model.hh>
// #include <gz/sim/Util.hh>
// #include <sdf/Element.hh>
// #include <sdf/Plugin.hh>

// #include "gz/sim/gui/Export.hh"
// #include "gz/sim/Entity.hh"
// #include "gz/sim/config.hh"


// // This header is required to register plugins. It's good practice to place it
// // in the cc file, like it's done here.
// #include <gz/plugin/Register.hh>

// // Don't forget to include the plugin's header.
// #include "mapping_gazebo/PtuFixedPosition.hh" 

// // This is required to register the plugin. Make sure the interfaces match
// // what's in tdhe header.
// GZ_ADD_PLUGIN(
//     mapping_gazebo::PtuFixedPosition,
//     gz::sim::System,
//     mapping_gazebo::PtuFixedPosition::ISystemConfigure,
//     mapping_gazebo::PtuFixedPosition::ISystemPreUpdate,
//     mapping_gazebo::PtuFixedPosition::ISystemUpdate,
//     mapping_gazebo::PtuFixedPosition::ISystemPostUpdate,
//     mapping_gazebo::PtuFixedPosition::ISystemReset
// )

// namespace mapping_gazebo 
// {

//   //PID controller for the joint
// 		gz::math::PID pid_pan;
// 		gz::math::PID pid_tilt;
// 		gz::math::PID pid_lidar;
  
//   //Pointer to the joint
// 		gz::sim::Entity joint_pan;
// 		gz::sim::Entity joint_tilt;
// 		gz::sim::Entity joint_lidar;
// 		gz::sim::Entity joint_imu;
// 		gz::sim::Entity joint_sensor;

//   //Pointer to model
//     gz::sim::Entity entity{gz::sim::kNullEntity};
//     gz::sim::Model model{gz::sim::kNullEntity};
//     gz::sim::Entity modelLink{gz::sim::kNullEntity};


// PtuFixedPosition::PtuFixedPosition()
//   : dataPtr(std::make_unique<PtuFixedPositionPrivate>())
// {
// }



// void PtuFixedPosition::Configure(const gz::sim::Entity &_entity,
//                 const std::shared_ptr<const sdf::Element> &_element,
//                 gz::sim::EntityComponentManager &_ecm,
//                 gz::sim::EventManager &_eventManager)
// {
//   //save entity, model, and name
//   entity = _entity;
//   model = gz::sim::Model(_entity);
//   modelName = this->dataPtr->model.Name(_ecm);
  
//   //Get Joints
//   control.joint = this->dataPtr->model.JointByName(_ecm, control.jointName);
    


// }

// void PtuFixedPosition::PreUpdate(const gz::sim::UpdateInfo &_info,
//                            gz::sim::EntityComponentManager &_ecm)
// {
//   if (!_info.paused && _info.iterations % 1000 == 0)
//   {
//     gzdbg << "mapping_gazebo::PtuFixedPosition::PreUpdate" << std::endl;
//   }
// }



// void PtuFixedPosition::PostUpdate(const gz::sim::UpdateInfo &_info,
//                             const gz::sim::EntityComponentManager &_ecm) 
// {
//     // This is a simple example of how to get information from UpdateInfo.
//   std::string msg = "Hello, world! Simulation is ";
//   if (!_info.paused)
//     msg += "not ";
//   msg += "paused.";

//   // Messages printed with gzmsg only show when running with verbosity 3 or
//   // higher (i.e. gz sim -v 3)
//   gzmsg << msg << std::endl;
// }

// }  // namespace mapping_gazebo
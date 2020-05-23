/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_mavlink_interface.h"

#include <ignition/plugin/Register.hh>

namespace gazebo_mavlink_interface
{

GazeboMavlinkInterface::GazeboMavlinkInterface()
{
}

GazeboMavlinkInterface::~GazeboMavlinkInterface()
{
}

void GazeboMavlinkInterface::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem::PostUpdate" << std::endl;
}

void GazeboMavlinkInterface::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  
}

void GazeboMavlinkInterface::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem::PostUpdate" << std::endl;
}
}
// Register plugin
IGNITION_ADD_PLUGIN(gazebo_mavlink_interface::GazeboMavlinkInterface, ignition::gazebo::System, gazebo_mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
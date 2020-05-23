/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
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

#include <vector>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <ignition/gazebo/System.hh>

#include <ignition/math.hh>
#include <sdf/sdf.hh>

namespace gazebo_mavlink_interface
{
class GazeboMavlinkInterface:
  // This class is a system.
  public ignition::gazebo::System,
  // This class also implements the ISystemPreUpdate, ISystemUpdate,
  // and ISystemPostUpdate interfaces.
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
  public:
    GazeboMavlinkInterface();
    ~GazeboMavlinkInterface() override;
    
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;
    void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
              const ignition::gazebo::EntityComponentManager &_ecm) override;
};
}

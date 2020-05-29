#include "gazebo_mavlink_interface.h"

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
    mavlink_interface::GazeboMavlinkInterface,
    ignition::gazebo::System,
    mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)
using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface()
{
  std::cout << "HelloWorld" << std::endl;
}

GazeboMavlinkInterface::~GazeboMavlinkInterface()
{
}

void GazeboMavlinkInterface::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
}
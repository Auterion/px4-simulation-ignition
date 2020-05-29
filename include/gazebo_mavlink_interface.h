#ifndef MAVLINK_INTERFACE_HH_
#define MAVLINK_INTERFACE_HH_

#include <ignition/gazebo/System.hh>

namespace mavlink_interface
{
  class GazeboMavlinkInterface:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: GazeboMavlinkInterface();

    public: ~GazeboMavlinkInterface() override;

    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}

#endif
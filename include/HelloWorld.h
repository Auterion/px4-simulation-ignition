#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
  class HelloWorld:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemPostUpdate
  {
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}
}
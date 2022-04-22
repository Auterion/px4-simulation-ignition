#include <string>
#include <ignition/common/Console.hh>

#include "HelloWorld.hh"

using namespace ignition;
using namespace gazebo;

void HelloWorld::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";
  
  ignmsg << msg << std::endl;
}


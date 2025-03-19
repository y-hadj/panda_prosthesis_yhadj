#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

#include <mc_panda/panda.h>

namespace mc_robots
{

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  for(auto & robot : {"PandaDefault"})
  {
    for(auto & tool : {"BoneTag::Femur", "BoneTag::Tibia"})
    {
      cb(robot, tool);
    }
  }
}

} // namespace mc_robots

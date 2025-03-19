#include "module.h"

#include <mc_rbdyn/RobotLoader.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    mc_robots::ForAllVariants([&names](const std::string & robotModule, const std::string & toolModule)
                              { names.push_back(robotModule + "::" + toolModule); });
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaProsthesis")
    static auto variant_factory = []()
    {
      std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> variant_factory;
      using namespace mc_robots;
      ForAllVariants(
          [&variant_factory](const std::string & robotModule, const std::string & toolModule)
          {
            auto variant_name = robotModule + "::" + toolModule;
            variant_factory[variant_name] = [=]()
            {
              auto name = toolModule == "BoneTag::Femur" ? "panda_femur" : "panda_tibia";
              auto robot_rm = *mc_rbdyn::RobotLoader::get_robot_module(robotModule);
              auto femur_rm = *mc_rbdyn::RobotLoader::get_robot_module(toolModule);
              return new mc_rbdyn::RobotModule(robot_rm.connect(
                  femur_rm, "panda_link8", "base_link", "", mc_rbdyn::RobotModule::ConnectionParameters{}.name(name)));
            };
          });
      return variant_factory;
    }();

    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("HRP4CR module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}

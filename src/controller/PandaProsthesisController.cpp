#include "PandaProsthesisController.h"
#include <mc_panda/devices/Robot.h>
#include <mc_rbdyn/RobotLoader.h>

namespace panda_prosthetics
{

PandaProsthetics::PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  robots().reserve(100);
  realRobots().reserve(100);
  outputRobots().reserve(100);

  auto robotConfig = config("robots")(robot().name());
  if(robotConfig.has("CollisionBehavior"))
  {
    auto colC = robotConfig("CollisionBehavior");
    mc_rtc::log::warning("[{}] Changing robot CollisionBeaviour to:\n{}", this->name_, colC.dump(true, true));
    auto & robot_device = robot().device<mc_panda::Robot>("Robot");
    robot_device.setCollisionBehavior(colC("lower_torque_thresholds").operator std::array<double, 7>(),
                                      colC("upper_torque_thresholds").operator std::array<double, 7>(),
                                      colC("lower_force_thresholds").operator std::array<double, 6>(),
                                      colC("upper_force_thresholds").operator std::array<double, 6>());
  }

  gui()->addElement(
      {"Frames"}, mc_rtc::gui::Transform("Tibia", [this]() { return robot("panda_tibia").frame("Tibia").position(); }),
      mc_rtc::gui::Transform("Femur", [this]() { return robot("panda_femur").frame("Femur").position(); }),
      mc_rtc::gui::Transform("TibiaCalibration",
                             [this]() { return robot("panda_tibia").frame("TibiaCalibration").position(); }),
      mc_rtc::gui::Transform("FemurCalibration",
                             [this]() { return robot("panda_femur").frame("FemurCalibration").position(); }));
  datastore().make<std::string>("ControllerName", config_("ControllerName", std::string{"bonetag"}));
}

bool PandaProsthetics::run()
{
  return mc_control::fsm::Controller::run();
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
}

void PandaProsthetics::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("PandaProsthesis", panda_prosthetics::PandaProsthetics)

} // namespace panda_prosthetics

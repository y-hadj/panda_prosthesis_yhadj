#include "PandaProsthesisController.h"
#include <mc_rbdyn/RobotLoader.h>
#include "config.h"

namespace panda_prosthetics
{

static auto patch_rm(mc_rbdyn::RobotModulePtr rm)
{
  auto bonetag_femur = mc_rbdyn::RobotLoader::get_robot_module("BoneTag::Femur");
  *rm = rm->connect(*bonetag_femur, "panda_link8", "base_link", "",
                    mc_rbdyn::RobotModule::ConnectionParameters{}.name("panda_femur"));
  return rm;
}

PandaProsthetics::PandaProsthetics(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(patch_rm(rm), dt, config), config_(config)
{
  gui()->addElement(
      {"Frames"}, mc_rtc::gui::Transform("Tibia", [this]() { return robot("panda_tibia").frame("Tibia").position(); }),
      mc_rtc::gui::Transform("Femur", [this]() { return robot("panda_femur").frame("Femur").position(); }),
      mc_rtc::gui::Transform("TibiaCalibration",
                             [this]() { return robot("panda_tibia").frame("TibiaCalibration").position(); }),
      mc_rtc::gui::Transform("FemurCalibration",
                             [this]() { return robot("panda_femur").frame("FemurCalibration").position(); }));
  datastore().make<std::string>("ControllerName", "bonetag");
}

bool PandaProsthetics::run()
{
  // return mc_control::fsm::Controller::run();
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
}

void PandaProsthetics::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("PandaProsthesis", panda_prosthetics::PandaProsthetics)

} // namespace panda_prosthetics

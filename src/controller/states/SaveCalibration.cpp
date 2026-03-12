#include "SaveCalibration.h"
#include <mc_control/fsm/Controller.h>
#include <utils.h>

void save(const std::string & etc_file, const mc_rbdyn::Robot & robot)
{
  mc_rtc::Configuration initial(etc_file);
  initial.add(robot.name());
  initial(robot.name()).add("pose", robot.posW());
  initial(robot.name()).add("joints", robot.mbc().q);
  initial.save(etc_file);
  mc_rtc::log::success("Calibration saved to {}", etc_file);
}

void SaveCalibration::start(mc_control::fsm::Controller & ctl)
{
  auto controllerName = ctl.datastore().get<std::string>("ControllerName");
  auto etc_dir = get_or_create_dir("calibration/" + controllerName);
  auto & robot = ctl.robot(config_("robot"));
  auto etc_file = etc_dir + "/initial_" + robot.name() + ".yaml";
  save(etc_file, robot);
  output("OK");
}

bool SaveCalibration::run(mc_control::fsm::Controller & ctl)
{
  return true;
}

void SaveCalibration::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("SaveCalibration", SaveCalibration)

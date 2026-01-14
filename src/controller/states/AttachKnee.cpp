#include "AttachKnee.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <SpaceVecAlg/PTransform.h>

void AttachKnee::start(mc_control::fsm::Controller & ctl)
{
  output("OK");
  auto & kneeBraceRobot = ctl.loadRobot(mc_rbdyn::RobotLoader::get_robot_module("BoneTag::KneeBrace"), "knee_brace");
  auto & tibiaRobot = ctl.robot("panda_tibia");
  if(kneeBraceRobot.hasFrame("KneeBraceCenter") && tibiaRobot.hasFrame("Tibia"))
  {
    kneeBraceRobot.posW(
        kneeBraceRobot.frame("KneeBraceCenter").X_b_f().inv() * ctl.robot("panda_tibia").frame("Tibia").position());
  }
}

bool AttachKnee::run(mc_control::fsm::Controller & ctl)
{
  if(config_("followKnee", true))
  {
    auto & kneeBraceRobot = ctl.robot("knee_brace");
    auto & tibiaRobot = ctl.robot("panda_tibia");
    if(kneeBraceRobot.hasFrame("KneeBraceCenter") && tibiaRobot.hasFrame("Tibia"))
    {
      kneeBraceRobot.posW(
        kneeBraceRobot.frame("KneeBraceCenter").X_b_f().inv() * ctl.robot("panda_tibia").frame("Tibia").position());
    }
  }
  return true;
};

void AttachKnee::teardown(mc_control::fsm::Controller & ctl)
{
};

EXPORT_SINGLE_STATE("AttachKnee", AttachKnee)

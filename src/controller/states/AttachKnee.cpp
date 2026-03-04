#include "AttachKnee.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <SpaceVecAlg/PTransform.h>

void AttachKnee::start(mc_control::fsm::Controller & ctl)
{
  output("OK");
  if(config_("display", true))
  {
    auto & kneeBraceRobot = ctl.loadRobot(mc_rbdyn::RobotLoader::get_robot_module("BoneTag::KneeBrace"), "knee_brace");
    auto & tibiaRobot = ctl.robot("panda_tibia");
    if(kneeBraceRobot.hasFrame("KneeBraceCenter") && tibiaRobot.hasFrame("Tibia"))
    {
      kneeBraceRobot.posW(kneeBraceRobot.frame("KneeBraceCenter").X_b_f().inv()
                          * ctl.robot("panda_tibia").frame("Tibia").position());
    }
  }

  collisions_added_ = false;
}

bool AttachKnee::run(mc_control::fsm::Controller & ctl)
{
  // add cols only on 1st run (i.e. after robot is fully loaded)
  if(!collisions_added_ && config_.has("AddCollisions"))
  {
    auto constraints = config_("AddCollisions");
    for(const auto & c : constraints)
    {
      auto constraint = mc_solver::ConstraintSetLoader::load<mc_solver::CollisionsConstraint>(ctl.solver(), c);
      ctl.solver().addConstraintSet(*constraint);
      added_constraints_.push_back(constraint);
    }
    collisions_added_ = true;
  }

  if(!config_("display", true)) return true;
  if(config_("followKnee", true))
  {
    auto & kneeBraceRobot = ctl.robot("knee_brace");
    auto & tibiaRobot = ctl.robot("panda_tibia");
    if(kneeBraceRobot.hasFrame("KneeBraceCenter") && tibiaRobot.hasFrame("Tibia"))
    {
      kneeBraceRobot.posW(kneeBraceRobot.frame("KneeBraceCenter").X_b_f().inv()
                          * ctl.robot("panda_tibia").frame("Tibia").position());
    }
  }
  return true;
};

void AttachKnee::teardown(mc_control::fsm::Controller & ctl)
{
  for(auto & constraint : added_constraints_)
  {
    ctl.solver().removeConstraintSet(*constraint);
  }
  added_constraints_.clear();
};

EXPORT_SINGLE_STATE("AttachKnee", AttachKnee)

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_solver/CollisionsConstraint.h>

struct AttachKnee : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    bool collisions_added_ = false;
    std::vector<std::shared_ptr<mc_solver::CollisionsConstraint>> added_constraints_;
  };

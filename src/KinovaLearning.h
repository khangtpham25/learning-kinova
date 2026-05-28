#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/TransformTask.h>

#include "api.h"

struct KinovaLearning_DLLAPI KinovaLearning : public mc_control::MCController
{
  KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);
  bool run() override;
  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  std::shared_ptr<mc_tasks::TransformTask> dingo_base_task_;
};

#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct KinovaLearning_DLLAPI KinovaLearning : public mc_control::MCController
{
  KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
};
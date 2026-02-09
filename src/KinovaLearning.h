#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct KinovaLearning_DLLAPI KinovaLearning : public mc_control::MCController
{
  KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  private:
    std::map< std::string, std::vector< double >> defaultPosture = {
      {"joint_1", {0}},
      {"joint_2", {0}},
      {"joint_3", {M_PI}},
      {"joint_4", {-M_PI * 3 / 4}},
      {"joint_5", {0}},
      {"joint_6", {M_PI / 4}}
    };
};
#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

enum ControlState 
{
  IDLE = 0,
  MOVE_UP,
  MOVE_DOWN
};

struct KinovaLearning_DLLAPI KinovaLearning : public mc_control::MCController
{
  KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  private:
    void switchState();
    ControlState state_ = IDLE;
    std::map< std::string, std::vector< double >> defaultPosture = {
      {"joint_1", {0}},
      {"joint_2", {0}},
      {"joint_3", {M_PI}},
      {"joint_4", {-M_PI * 3 / 4}},
      {"joint_5", {0}},
      {"joint_6", {M_PI / 4}}
    };
    double moveMag = M_PI / 6;
    std::map< std::string, std::vector< double >> upPosture = 
    {
      {"joint_6", {M_PI / 4 + moveMag}}
    };
    std::map< std::string, std::vector< double >> downPosture = 
    {
      {"joint_6", {M_PI / 4 - moveMag}}
    };
};

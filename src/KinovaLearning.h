#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

enum ControlState
{
  IDLE = 0,
  MOVE_UP,
  MOVE_DOWN
};

enum StoreState
{
  START = 0,
  FOLD,
  BEND
};

struct KinovaLearning_DLLAPI KinovaLearning : public mc_control::MCController
{
  KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);
  bool run() override;
  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  // ControlState
  void switchState();
  ControlState state_ = IDLE;
  std::map<std::string, std::vector<double>> default_posture_ = {
      {"joint_1", {1.5 * M_PI}}, {"joint_2", {0}},           {"joint_3", {M_PI}}, {"joint_4", {-0.75 * M_PI}},
      {"joint_5", {0}},          {"joint_6", {0.25 * M_PI}}, {"joint_7", {M_PI}}};
  double move_mag_ = M_PI / 6;
  std::map<std::string, std::vector<double>> up_posture_ = {{"joint_6", {M_PI / 4 + move_mag_}}};
  std::map<std::string, std::vector<double>> down_posture_ = {{"joint_6", {M_PI / 4 - move_mag_}}};

  // StoreState
  void storeState();
  StoreState store_state_ = START;
  std::map<std::string, std::vector<double>> store_pose_start_ = {
      {"joint_1", {0.5 * M_PI}}, {"joint_2", {0}},   {"joint_3", {0}}, {"joint_4", {0}},
      {"joint_5", {0}},          {"joint_6", {2.0}}, {"joint_7", {0}}};
  std::map<std::string, std::vector<double>> store_pose_fold_ = {{"joint_4", {2.3}}, {"joint_5", {0.4}}};
  std::map<std::string, std::vector<double>> store_pose_bend_ = {{"joint_1", {0.0}}, {"joint_2", {0.5 * M_PI}}};
};

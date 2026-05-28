#include "KinovaLearning.h"

KinovaLearning::KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(std::move(rm), dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(compoundJointConstraint);
  postureTask->stiffness(0.5);
  postureTask->weight(1);
  solver().addTask(postureTask);

  mc_rtc::log::success("KinovaLearning init done ");
}

bool KinovaLearning::run()
{
  bool storing = true;
  if (storing)
  {
    if (postureTask->eval().norm() < 0.05 && postureTask->speed().norm() < 0.05)
    {
      storeState();
    }
  }
  else
  {
    if (postureTask->eval().norm() < 0.02 && postureTask->speed().norm() < 0.01)
    {
      switchState();
    }
  }
  return mc_control::MCController::run();
}

void KinovaLearning::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void KinovaLearning::storeState()
{
  switch (store_state_)
  {
  case START:
    postureTask->target(store_pose_start_);
    store_state_ = FOLD;
    break;

  case FOLD:
    postureTask->target(store_pose_fold_);
    store_state_ = BEND;
    break;

  case BEND:
    postureTask->target(store_pose_bend_);
    // state_ = MOVE_UP;
    break;
  }
}

void KinovaLearning::switchState()
{
  switch (state_)
  {
  case IDLE:
    postureTask->target(default_posture_);
    state_ = MOVE_UP;
    break;

  case MOVE_UP:
    postureTask->target(up_posture_);
    state_ = MOVE_DOWN;
    break;

  case MOVE_DOWN:
    postureTask->target(down_posture_);
    state_ = MOVE_UP;
    break;
  }
}

CONTROLLER_CONSTRUCTOR("KinovaLearning", KinovaLearning)

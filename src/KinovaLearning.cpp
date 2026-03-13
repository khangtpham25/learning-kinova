#include "KinovaLearning.h"

KinovaLearning::KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
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
  if(storing)
  {
    if(postureTask->eval().norm() < 0.05 && postureTask->speed().norm() < 0.05)
    {
      storeState();
    }
  }
  else
  {
    if(postureTask->eval().norm() < 0.02 && postureTask->speed().norm() < 0.01)
    {
      switchState();
    }
  }
  return mc_control::MCController::run();
}

void KinovaLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void KinovaLearning::storeState()
{
  switch(store_state_)
  {
    case START:
      postureTask->target(storePoseStart);
      store_state_ = FOLD;
      break;

    case FOLD:
      postureTask->target(storePoseFold);
      store_state_ = BEND;
      break;

    case BEND:
      postureTask->target(storePoseBend);
      // state_ = MOVE_UP;
      break;
  }
}

void KinovaLearning::switchState()
{
  switch(state_)
  {
    case IDLE:
      postureTask->target(defaultPosture);
      state_ = MOVE_UP;
      break;

    case MOVE_UP:
      postureTask->target(upPosture);
      state_ = MOVE_DOWN;
      break;

    case MOVE_DOWN:
      postureTask->target(downPosture);
      state_ = MOVE_UP;
      break;
  }
}

CONTROLLER_CONSTRUCTOR("KinovaLearning", KinovaLearning)

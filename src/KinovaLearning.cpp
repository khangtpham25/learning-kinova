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
  postureTask->target(defaultPosture);
  return mc_control::MCController::run();
}

void KinovaLearning::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("KinovaLearning", KinovaLearning)

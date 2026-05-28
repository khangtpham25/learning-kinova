#include "KinovaLearning.h"

#include <mc_rbdyn/RobotLoader.h>

KinovaLearning::KinovaLearning(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController({std::move(rm), mc_rbdyn::RobotLoader::get_robot_module("dingo"),
                                mc_rbdyn::RobotLoader::get_robot_module("env/ground")},
                               dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  postureTask->stiffness(0.5);
  postureTask->weight(1);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  double friction = mc_rbdyn::Contact::defaultFriction;
  dof[2] = 0.0;
  dof[3] = 0.0;
  dof[4] = 0.0;
  addContact({"dingo", "ground", "Base", "AllGround", friction, dof});
  addContact({"dingo", "kinova", "Base", "Base"});

  mc_rtc::log::success("KinovaLearning init done ");
}

bool KinovaLearning::run()
{
  return mc_control::MCController::run();
}

void KinovaLearning::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);

  robots().robot("kinova").posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.5)));
  robots().robot("dingo").posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(-0.25, 0.0, 0.0)));

  dingo_base_task_ = std::make_shared<mc_tasks::TransformTask>("base_link", robots(), 1, 2.0, 1000);

  solver().addTask(dingo_base_task_);
  postureTask->target({{"shoulder_lift_joint", {-M_PI / 2}}});
  dingo_base_task_->target({Eigen::Vector3d(1.5, 0.0, 0.0)});
}

CONTROLLER_CONSTRUCTOR("KinovaLearning", KinovaLearning)

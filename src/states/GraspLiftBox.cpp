#include "GraspLiftBox.h"

#include "../DemoController.h"

void GraspLiftBox::configure(const mc_rtc::Configuration & config)
{
    config("objectName", objectName_);
    config("objectSurfaceLeftGripper", objectSurfaceLeftGripper_);
    config("objectSurfaceRightGripper", objectSurfaceRightGripper_);
    config("approachOffsetZ", approachOffsetZ_);
    config("liftHeight", liftHeight_);
    config("completionEval", completionEval_);
    config("completionSpeed", completionSpeed_);
}

void GraspLiftBox::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController &>(ctl_);

    phase_ = Phase::Approach;
    contactAdded_ = false;

    leftGripperTask = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame("LeftHandWrench"), 5.0, 1000.0);
    ctl.solver().addTask(leftGripperTask);
    
    auto leftGripperTargetPose = ctl.robot(objectName_).frame(objectSurfaceLeftGripper_).position();
    leftGripperTargetPose.translation() += Eigen::Vector3d(0.0, 0.0, approachOffsetZ_);
    leftGripperTargetPose.rotation() = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    leftGripperTask->target(leftGripperTargetPose);
    
    rightGripperTask = std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame("RightHandWrench"), 5.0, 1000.0);
    ctl.solver().addTask(rightGripperTask);

    auto rightGripperTargetPose = ctl.robot(objectName_).frame(objectSurfaceRightGripper_).position();
    rightGripperTargetPose.translation() += Eigen::Vector3d(0.0, 0.0, approachOffsetZ_);
    rightGripperTask->target(rightGripperTargetPose);
}

bool GraspLiftBox::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController &>(ctl_);

    const bool completed = (
        leftGripperTask->eval().norm() < completionEval_ 
        && leftGripperTask->speed().norm() < completionSpeed_ 
        && rightGripperTask->eval().norm() < completionEval_ 
        && rightGripperTask->speed().norm() < completionSpeed_
    );

    if(!completed)
    {
        return false;
    }

    if(phase_ == Phase::Approach)
    {
        ctl.addContact({ctl.robot().name(), ctl.robot(objectName_).name(), "LeftHandWrench", objectSurfaceLeftGripper_});
        ctl.addContact({ctl.robot().name(), ctl.robot(objectName_).name(), "RightHandWrench", objectSurfaceRightGripper_});

        contactAdded_ = true;

        auto leftLiftTarget = leftGripperTask->target();
        leftLiftTarget.translation() += Eigen::Vector3d(0.0, 0.0, liftHeight_);
        leftGripperTask->target(leftLiftTarget);
        
        auto rightLiftTarget = rightGripperTask->target();
        rightLiftTarget.translation() += Eigen::Vector3d(0.0, 0.0, liftHeight_);
        rightGripperTask->target(rightLiftTarget);
        
        phase_ = Phase::Lift;
        return false;
    }

    if(phase_ == Phase::Lift)
    {
        phase_ = Phase::Done;
        output("OK");
        return true;
    }

    return false;
}

void GraspLiftBox::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(leftGripperTask);
    ctl.solver().removeTask(rightGripperTask);
    leftGripperTask.reset();
    rightGripperTask.reset();

    if(contactAdded_)
    {
        ctl.removeContact({ctl.robot().name(), ctl.robot(objectName_).name(), "LeftHandWrench", objectSurfaceLeftGripper_});
        ctl.removeContact({ctl.robot().name(), ctl.robot(objectName_).name(), "RightHandWrench", objectSurfaceRightGripper_});
        contactAdded_ = false;
    }
}

EXPORT_SINGLE_STATE("GraspLiftBox", GraspLiftBox)

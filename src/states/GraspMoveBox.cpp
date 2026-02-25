#include "GraspMoveBox.h"

#include <console_bridge/console.h>

#include "../DemoController.h"
#include "BaselineWalkingController/CentroidalManager.h"
#include "BaselineWalkingController/FootManager.h"

void GraspMoveBox::configure(const mc_rtc::Configuration &config)
{
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("graspFromPose", m_graspFromPose);
    config("dropFromPose", m_dropFromPose);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("approachOffset", m_approachOffset);
    config("liftHeight", m_liftHeight);
    config("liftPullback", m_liftPullback);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("raiseLeftHandPos", m_raiseLeftHandPos);
    config("raiseRightHandPos", m_raiseRightHandPos);
    config("raiseLeftHandOri", m_raiseLeftHandOri);
    config("raiseRightHandOri", m_raiseRightHandOri);
    config("raiseHandsStiffness", m_raiseHandsStiffness);
    config("raiseHandsWeight", m_raiseHandsWeight);
    config("raiseHandsCompletionEval", m_raiseHandsCompletionEval);
    config("raiseHandsCompletionSpeed", m_raiseHandsCompletionSpeed);
}

void GraspMoveBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.footManager_->walkToRelativePose(m_graspFromPose);

    mc_rtc::log::info("Now in walking phase");
    m_phase = Phase::Walking;
    m_contactAdded = false;
}

bool GraspMoveBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // This is a hack to ensure the object is visible in mc_mujoco because for some reason the
    // box position does not change in the visualization
    if (m_phase == Phase::Lift)
    {
        const auto setPosWCall = m_objectName + "::SetPosW";
        if (ctl.datastore().has(setPosWCall))
        {
            const auto &objectPosW = ctl.robot(m_objectName).posW();
            ctl.datastore().call<void, const sva::PTransformd &>(setPosWCall, objectPosW);
        }
    }

    if (m_phase == Phase::Walking && ctl.footManager_->footstepQueue().empty())
    {
        mc_rtc::log::info("Now in raise hands phase");
        m_phase = Phase::RaiseHands;

        const auto &leftFootPose = ctl.robot().frame("LeftFootCenter").position();
        const auto &rightFootPose = ctl.robot().frame("RightFootCenter").position();
        Eigen::Vector3d midPos = 0.5 * (leftFootPose.translation() + rightFootPose.translation());

        m_leftGripperTask = std::make_shared<mc_tasks::TransformTask>
        (
            ctl.robot().frame("LeftHandWrench"),
            m_raiseHandsStiffness,
            m_raiseHandsWeight
        );
        sva::PTransformd leftTarget;
        leftTarget.translation() = midPos + m_raiseLeftHandPos;
        leftTarget.rotation() = m_raiseLeftHandOri.toRotationMatrix();
        m_leftGripperTask->target(leftTarget);
        ctl.solver().addTask(m_leftGripperTask);

        m_rightGripperTask = std::make_shared<mc_tasks::TransformTask>
        (
            ctl.robot().frame("RightHandWrench"),
            m_raiseHandsStiffness,
            m_raiseHandsWeight
        );
        sva::PTransformd rightTarget;
        rightTarget.translation() = midPos + m_raiseRightHandPos;
        rightTarget.rotation() = m_raiseRightHandOri.toRotationMatrix();
        m_rightGripperTask->target(rightTarget);
        ctl.solver().addTask(m_rightGripperTask);
        return false;
    }

    const bool completed = (
        m_leftGripperTask->eval().norm() < m_completionEval
        && m_leftGripperTask->speed().norm() < m_completionSpeed
        && m_rightGripperTask->eval().norm() < m_completionEval
        && m_rightGripperTask->speed().norm() < m_completionSpeed
    );

    if (!completed)
    {
        return false;
    }

    if (m_phase == Phase::RaiseHands)
    {
        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::Approach;

        // Reconfigure tasks for approach
        m_leftGripperTask->stiffness(m_stiffness);
        m_leftGripperTask->weight(m_weight);

        auto leftGripperTargetPose = ctl.robot(m_objectName).frame
                (m_objectSurfaceLeftGripper).position();
        leftGripperTargetPose.translation() += ctl.robot().posW().rotation() * Eigen::Vector3d
                (0.0, m_approachOffset, 0.0);
        leftGripperTargetPose.rotation() = Eigen::Quaterniond
                (0.5, 0.5, 0.5, -0.5).toRotationMatrix();
        m_leftGripperTask->target(leftGripperTargetPose);

        m_rightGripperTask->stiffness(m_stiffness);
        m_rightGripperTask->weight(m_weight);

        auto rightGripperTargetPose = ctl.robot(m_objectName).frame
                (m_objectSurfaceRightGripper).position();
        rightGripperTargetPose.translation() += ctl.robot().posW().rotation() *
                Eigen::Vector3d(0.0, -m_approachOffset, 0.0);
        rightGripperTargetPose.rotation() = Eigen::Quaterniond
                (0.5, -0.5, 0.5, 0.5).toRotationMatrix();
        m_rightGripperTask->target(rightGripperTargetPose);
        return false;
    }

    if (m_phase == Phase::Approach)
    {
        auto previousLeftTarget = m_leftGripperTask->target();
        previousLeftTarget.translation() = ctl.robot(m_objectName)
                .frame(m_objectSurfaceLeftGripper)
                .position()
                .translation();
        m_leftGripperTask->target(previousLeftTarget);

        auto previousRightTarget = m_rightGripperTask->target();
        previousRightTarget.translation() = ctl.robot(m_objectName)
                .frame(m_objectSurfaceRightGripper)
                .position()
                .translation();
        m_rightGripperTask->target(previousRightTarget);

        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::Grasping;
        return false;
    }

    if (m_phase == Phase::Grasping)
    {
        auto leftContact = mc_control::Contact
        (
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "LeftHandWrench",
            m_objectSurfaceLeftGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones()
        );
        ctl.addContact(leftContact);

        auto rightContact = mc_control::Contact
        (
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "RightHandWrench",
            m_objectSurfaceRightGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones()
        );
        ctl.addContact(rightContact);

        m_contactAdded = true;

        auto leftLiftTarget = m_leftGripperTask->target();
        leftLiftTarget.translation() += Eigen::Vector3d(-m_liftPullback, 0.0, m_liftHeight);
        m_leftGripperTask->target(leftLiftTarget);

        auto rightLiftTarget = m_rightGripperTask->target();
        rightLiftTarget.translation() += Eigen::Vector3d(-m_liftPullback, 0.0, m_liftHeight);
        m_rightGripperTask->target(rightLiftTarget);

        mc_rtc::log::info("Now in lift phase");
        m_phase = Phase::Lift;
        return false;
    }

    if (m_phase == Phase::Lift)
    {
        mc_rtc::log::info("Done");
        m_phase = Phase::Done;
        output("OK");
        return true;
    }

    return false;
}

void GraspMoveBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
    m_leftGripperTask.reset();
    m_rightGripperTask.reset();

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.removeContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "LeftHandWrench",
                m_objectSurfaceLeftGripper
            }
        );
        ctl.removeContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "RightHandWrench",
                m_objectSurfaceRightGripper
            }
        );
        m_contactAdded = false;
    }
}

EXPORT_SINGLE_STATE("GraspMoveBox", GraspMoveBox)

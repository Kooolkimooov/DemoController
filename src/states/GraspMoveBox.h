#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

struct GraspMoveBox : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration &config) override;

    void start(mc_control::fsm::Controller &ctl) override;

    bool run(mc_control::fsm::Controller &ctl) override;

    void teardown(mc_control::fsm::Controller &ctl) override;

private:
    enum class Phase
    {
        Walking,
        RaiseHands,
        Approach,
        Grasping,
        Lift,
        Done
    };

    std::shared_ptr<mc_tasks::TransformTask> m_leftGripperTask, m_rightGripperTask;

    std::string m_objectName;
    std::string m_objectSurfaceLeftGripper, m_objectSurfaceRightGripper;

    Eigen::Vector3d m_graspFromPose, m_dropFromPose;

    double m_stiffness = 1.0;
    double m_weight = 1000.0;
    double m_approachOffset = 0.025;
    double m_liftHeight = 0.10;
    double m_liftPullback = 0.0;
    double m_completionEval = 0.05;
    double m_completionSpeed = 1e-4;

    Eigen::Vector3d m_raiseLeftHandPos = Eigen::Vector3d(0.0, 0.2, 0.8);
    Eigen::Vector3d m_raiseRightHandPos = Eigen::Vector3d(0.0, -0.2, 0.8);
    Eigen::Quaterniond m_raiseLeftHandOri = Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5);
    Eigen::Quaterniond m_raiseRightHandOri = Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5);
    double m_raiseHandsStiffness = 5.0;
    double m_raiseHandsWeight = 1000.0;
    double m_raiseHandsCompletionEval = 0.1;
    double m_raiseHandsCompletionSpeed = 1e-4;

    bool m_contactAdded = false;
    bool m_removeContactAtTeardown = true;
    Phase m_phase = Phase::Approach;
};

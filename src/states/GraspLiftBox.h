#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

struct GraspLiftBox : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    enum class Phase
    {
        Approach,
        Lift,
        Done
    };

    std::shared_ptr<mc_tasks::TransformTask> leftGripperTask, rightGripperTask;

    std::string objectName_;
    std::string objectSurfaceLeftGripper_, objectSurfaceRightGripper_;
    double approachOffsetZ_ = 0.025;
    double liftHeight_ = 0.10;
    double completionEval_ = 0.05;
    double completionSpeed_ = 1e-4;
    bool contactAdded_ = false;

    Phase phase_ = Phase::Approach;
};

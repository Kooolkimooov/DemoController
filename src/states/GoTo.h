#pragma once

#include <mc_control/fsm/State.h>

struct GoTo : mc_control::fsm::State
{
        bool run(mc_control::fsm::Controller &) override;
        void start(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

        // this remains to be implemented
        virtual void configure(const mc_rtc::Configuration &) override;

    protected:
        Eigen::Vector3d m_destinationPoseWorld;
        Eigen::Vector3d computeRelativePose(Eigen::Vector3d poseWorld, sva::PTransformd robotPoseWorld);
};

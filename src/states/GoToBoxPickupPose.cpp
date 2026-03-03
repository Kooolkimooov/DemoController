#include "GoToBoxPickupPose.h"

void GoToBoxPickupPose::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("\n{}", config.dump(true, true));

    config("graspFromPoseWorld", m_destinationPoseWorld);
}

EXPORT_SINGLE_STATE("GoToBoxPickupPose", GoToBoxPickupPose)

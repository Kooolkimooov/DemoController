#pragma once

#include "GoTo.h"

struct GoToBoxPickupPose : GoTo
{
    public:
        void configure(const mc_rtc::Configuration &) override;
};

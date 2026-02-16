#include "DemoController.h"

DemoController::DemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : BWC::BaselineWalkingController(rm, dt, config)
{
    for (const auto & k : config("robots").keys())
    {
        mc_rtc::log::info("DemoController config('robots'): {}", k);
        auto v = config("robots")(k);
        for (const auto & kk : v.keys())
        {
            mc_rtc::log::info("                               ('{}'): {}", k, kk);
        }
    }
    mc_rtc::log::success("DemoController init done ");
}

bool DemoController::run()
{
    return BWC::BaselineWalkingController::run();
}

void DemoController::reset(const mc_control::ControllerResetData &reset_data)
{
    BWC::BaselineWalkingController::reset(reset_data);
}

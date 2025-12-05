#include "Copter.h"
#include "Parameters.h"
#include <AP_AHRS/AP_AHRS.h>
#include "mode_drone_show.h"

#if MODE_DYNAMIC_RTL == ENABLED

bool ModeDynamicRtl::init(bool ignore_checks)
{
    initialization_start();
    gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] Dynamic_RTL init OK!\r\n");
    return true;
}

void ModeDynamicRtl::initialization_start()
{   
    if(!AP::ahrs().get_relative_position_NED_home((path_ys[0]))){
        gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] ERROR:generate_path\r\n");
    }
    path_ys[0].z = 400.0f;

    wp_nav->wp_and_spline_init();
    wp_nav->set_wp_destination(path_ys[0], false);
    // auto_yaw.set_mode_to_default(false);
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    return;
}

void ModeDynamicRtl::run()
{
    if (wp_nav->reached_wp_destination()){
        gcs().send_text(MAV_SEVERITY_INFO, "[Serein_Y] Performance OK!!!\r\n");   
        copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);  
    }
    pos_control_run();
}

void ModeDynamicRtl::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();
    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}
#endif
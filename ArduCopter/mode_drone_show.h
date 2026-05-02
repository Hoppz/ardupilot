#pragma once

#include <AC_DroneShowManager/AC_DroneShowManager.h>

// Provide Copter-specific implementation of the drone show mode. While most of
// the logic for performing a show is present in AC_DroneShowManager, this class
// allows Copter to override base functionality - for example, to switch flight
// mode when the show is authorized.
class AC_DroneShowManager_Copter : public AC_DroneShowManager {
public:

    using AC_DroneShowManager::AC_DroneShowManager;

    /* Do not allow copies */
    AC_DroneShowManager_Copter(const AC_DroneShowManager_Copter &other) = delete;
    AC_DroneShowManager_Copter &operator=(const AC_DroneShowManager_Copter&) = delete;

    virtual bool get_current_location(Location& loc) const override;
    virtual bool get_current_relative_position_NED_origin(Vector3f& vec) const override;
    virtual void _request_switch_to_show_mode() override;

};

class ModeDroneShow : public Mode {


public:
    ModeDroneShow();
    Number mode_number() const override { return Number::DRONE_SHOW; }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;
    virtual void exit() override;

    bool requires_GPS() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool use_pilot_yaw() const override;
    /* in_guided_mode() should not return true because that would allow
     * scripting or GCS commands to mess around with the show execution */

    /* Make sure that the drone does not react to manual throttle changes.
     * This also disallows disarming with the rudder while the drone is in
     * show mode but not flying */
    bool has_manual_throttle() const override { return false; }

    bool is_landing() const override;
    bool is_taking_off() const override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "DRONE_SHOW"; }
    const char *name4() const override { return "SHOW"; }

    // customize takeoff behaviour to be mostly identical to guided mode
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override;

private:

    // --- Internal variables ---

    // Execution stage of the show
    DroneShowModeStage _stage;

    // Stores whether we have attempted to start the motors, due 10 seconds
    // before takeoff. Note that it does _not_ indicate whether the motors are
    // actually _running_.
    bool _motors_started;

    // Stores the timestamp when we have last attempted to set the home position
    // to the current location. Used to reset the home position every 30 seconds
    // during the "waiting for start time" phase to keep the AGL measurement at
    // zero.
    uint32_t _last_home_position_reset_attempt_at;

    // Stores the timestamp when we have changed the execution stage the
    // last time
    uint32_t _last_stage_change_at;

    // Stores whether we have set the home position to the takeoff position
    // before takeoff.
    bool _home_position_set;

    // Stores which stage to step to after the takeoff has completed. This
    // distinguishes "test takeoff" instructed from the GCS with a takeoff command
    // from "live takeoff", which happens when the start time is reached.
    DroneShowModeStage _next_stage_after_takeoff;

    // Stores whether we have performed the preflight calibration before takeoff.
    bool _preflight_calibration_done;

    // Timestamp until we block arming attempts during the startup phase if we
    // have attempted to arm the drone recently
    uint32_t _prevent_arming_until_msec;

    // Flag that stores whether the drone is limited to move only above the
    // takeoff altitude. The flag is set when entering the "performing" stage;
    // the limitation is relaxed when the real trajectory of the drone rises
    // above this altitude. The role of this limitation is to prevent the drone
    // from temporarily sinking below the takeoff altitude when the "real"
    // takeoff in the show trajectory is slower.
    bool _altitude_locked_above_takeoff_altitude;

    /// hoppz
    // Timestamp when landing started, used for delaying RTL command
    uint32_t _landing_start_time_ms;
    /// hoppz

    // Sets the stage of the execution to the given value
    void _set_stage(DroneShowModeStage value);

    bool cancel_requested() const;
    int32_t get_default_yaw_cd() const;
    int32_t get_elapsed_time_since_last_home_position_reset_attempt_msec() const;
    int32_t get_elapsed_time_since_last_stage_change_msec() const;

    void check_changes_in_parameters();
    void notify_authorization_changed();
    void notify_start_time_changed();
    bool send_guided_mode_command_during_performance();
    bool start_motors_if_not_running() WARN_IF_UNUSED;
    bool try_to_update_home_position();
    bool try_to_start_motors_if_prepared_to_take_off();

    void initialization_start();
    void initialization_run();

    void wait_for_start_time_start();
    void wait_for_start_time_run();

    void takeoff_start();
    void takeoff_run();
    bool takeoff_completed() const;
    bool takeoff_timed_out() const;

    void performing_start();
    void performing_run();
    bool performing_completed() const;

    // Two-phase trigger state for dynamic-RTL detection.
    // 0 = waiting for altitude above TRIG_ALT_CM (relative to home),
    // 1 = altitude crossed, waiting for descent through hysteresis to trigger.
    uint8_t status_flag;

    // Target position for dynamic-RTL navigation phase (home + 2m), stored in
    // NEU cm relative to the EKF origin (same frame as pos_control targets).
    Vector3f _dyn_rtl_target_neu_cm;

    // Debounce counter for optional immediate disarm when altitude matches home
    // within a tight tolerance during Dynamic-RTL land (see cpp for guards).
    uint8_t _dyn_rtl_home_alt_disarm_debounce;

    // Returns true the moment the two-phase altitude check fires (drone has
    // risen above a trigger altitude and then descended back past it with
    // hysteresis). Used to kick off the internal dynamic-RTL flow.
    bool check_reaching_rtl_altitude();

    void landing_start();
    void landing_run();
    bool landing_completed() const;

    // Dynamic-RTL internal sub-stages (replaces the old DYNAMIC_RTL flight mode).
    // Stage 1: fly to home + 2m using pos_control (no wp_nav, no controller reset).
    void dynamic_rtl_nav_start();
    void dynamic_rtl_nav_run();
    bool dynamic_rtl_nav_completed() const;

    // Stage 2: smooth land from home + 2m. Reuses already-active pos_control
    // and the base-class land_run_normal_or_precland() to avoid any re-init.
    void dynamic_rtl_land_start();
    void dynamic_rtl_land_run();
    bool dynamic_rtl_land_completed() const;

    void rtl_start();
    void rtl_run();
    bool rtl_completed() const;

    void loiter_start();
    void loiter_run();

    void landed_start();
    void landed_run();

    void error_start();
    void error_run();
};

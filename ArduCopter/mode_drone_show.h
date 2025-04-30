#pragma once

#include <AC_DroneShowManager/AC_DroneShowManager.h>

//! 在主程序中使用 1hz loop 发送舞步状态
#define USERHOOK_SUPERSLOWLOOP

// Provide Copter-specific implementation of the drone show mode. While most of
// the logic for performing a show is present in AC_DroneShowManager, this class
// allows Copter to override base functionality - for example, to switch flight
// mode when the show is authorized.
class AC_DroneShowManager_Copter : public AC_DroneShowManager {
public:

    using AC_DroneShowManager::AC_DroneShowManager;

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

    bool requires_GPS() const override {return true;}
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override {return true;}
    bool has_user_takeoff(bool must_navigate) const override {return true;}
    bool use_pilot_yaw() const override;

    //! 不能手动设置油门
    bool has_manual_throttle() const override {return false;}

    bool is_landing() const override;
    bool is_taking_off() const override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    const char* name() const override { return "DRONE_SHOW"; }
    const char* name4() const override { return "SHOW"; }

    // 自定义起飞行为，guided mode    
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    // 给地面站汇报
    bool get_wp(Location &loc) const override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const  override;
    float crosstrack_error() const override;

private:
    // --- 内部变量 ---

    // 当前的阶段
    DroneShowModeStage _stage;

    // 存储何时设置的当前home position 
    // `waiting for start time` 阶段30s 重置一次   
    // 以保持AGL(Above Ground Level)测量为零 
    uint32_t _last_home_position_reset_attempt_at;

    // 存储何时变为的当前阶段
    uint32_t _last_stage_change_at;

    // 在这个时间点之前，系统会拒绝 arming 
    uint32_t _prevent_arming_until_msec;

    // 存储起飞完成后该进入哪个阶段 
    //! 这将GCS指示的“测试起飞”与起飞命令与“实时起飞”区分开来，后者在到达开始时间时发生   
    DroneShowModeStage _next_stage_after_takeoff;

    // motors 是否已启动
    bool _motors_started;

    // 是否已设置 home position
    bool _home_position_set;

    // 是否完成起飞前检测
    bool _preflight_calibration_done;
    // true -> 锁定高度
    bool _altitude_locked_above_takeoff_altitude;

    void _set_stage(DroneShowModeStage val);

    bool cancel_requested() const;
    int32_t get_default_yaw_cd() const;
    uint32_t get_elapsed_time_since_last_home_position_reset_attempt_msec() const;
    uint32_t get_elapsed_time_since_last_stage_change_msec() const;

    void check_change_in_parameters();
    void notify_start_time_changed();
    void notify_authorization_changed();
    bool send_guided_mode_command_during_performance();
    bool start_motors_if_not_running() WARN_IF_UNUSED;
    bool try_to_update_home_position();
    bool try_to_start_motors_if_prepared_to_take_off();

    // init
    void initialization_start();
    void initialization_run();

    // wait for start
    void wait_for_start_time_start();
    void wait_for_start_time_run();

    // take off 
    void takeoff_start();
    void takeoff_run();
    bool takeoff_completed() const;
    bool takeoff_timed_out() const;
    
    // perform
    void performing_start();
    void performing_run();
    bool performing_completed() const;
	
	// rtl
    void rtl_start();
    void rtl_run();
    bool rtl_completed() const;
	
	// loiter
    void loiter_start();
    void loiter_run();
    
    // landing
    void landing_start();
    void landing_run();
    bool landing_completed() const;
    
    void landed_start();
    void landed_run();

    void error_start();
    void error_run();
};




#include "Copter.h"
#include "skybrush/skybrush.h"

#if MODE_GUIDED_ENABLED == ENABLED

//* AC_DroneShowManager_Copter
bool AC_DroneShowManager_Copter::get_current_location(Location& loc)  const
{
    // ahrs 姿态航向系统 (姿态估计中包含了位置估计)
    return copter.ahrs.get_location(loc);
}

bool AC_DroneShowManager_Copter::get_current_relative_position_NED_origin(Vector3f& vec) const
{
    return copter.ahrs.get_relative_position_NED_origin(vec);
}

void AC_DroneShowManager_Copter::_request_switch_to_show_mode()
{
    // 当 motor disarmed 才能切换
    if( !copter.motors->armed() ){
        copter.set_mode(Mode::Number::DRONE_SHOW,ModeReason::SCRIPTING);
    }
}

//* mode_drone_show
ModeDroneShow::ModeDroneShow(void): 
    Mode(),
    _stage(DroneShow_Off),
    _last_home_position_reset_attempt_at(0),
    _last_stage_change_at(0)
{}

bool ModeDroneShow::init(bool ignore_checks)
{
    initialization_start();
    return true;
}


// 至少以 25hz 调用, 实际上运行在 400hz
// update_flight_mode: flightmode->run()
void ModeDroneShow::run()
{
    check_change_in_parameters();
    
    switch (_stage) {
        case DroneShow_Init:
            // mode has just been initialized
            initialization_run();
            break;
        
        case DroneShow_WaitForStartTime:
            // waiting for start time
            wait_for_start_time_run();
            break;

        case DroneShow_Takeoff:
            // taking off
            takeoff_run();
            break;
        
        case DroneShow_Performing:
            // performing show
            performing_run();
            break;
        case DroneShow_Landing:
            // landing at the end of the show (normal termination)
            landing_run();
            break;

        case DroneShow_RTL:
            // returning to home position (abnormal termination)
            rtl_run();
            break;

        case DroneShow_Loiter:
            // holding position (joined show while airborne)
            loiter_run();
            break;

        case DroneShow_Landed:
            // landed successfully after a show
            landed_run();
            break;

        case DroneShow_Error:
            // failed to start a show
            error_run();
            break;

        default:
            break;
    }
}

void ModeDroneShow::exit()
{
    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // 清除所有依赖 start time 的状态信息
    notify_start_time_changed();

    // 设置当前的阶段为 "off"
    _set_stage(DroneShowModeStage::DroneShow_Off);
    
    // 同时 drone show manager 已退出 drone show mode
    copter.g2.drone_show_manager.notify_drone_show_mode_exited();
}

//* 判断是否允许解锁
bool ModeDroneShow::allows_arming(AP_Arming::Method method) const
{
    return (
        // 只有当请求来自地面站 或者
        // 同时满足
        // 1. 成功加载了表演数据
        // 2. 在有效的起飞时间
        // 3. 用户设置了表演原点
        // 4. 用户设置了表演的方西
        method == AP_Arming::Method::MAVLINK || (
            copter.g2.drone_show_manager.loaded_show_data_successfully() &&
            copter.g2.drone_show_manager.has_valid_takeoff_time() &&
            copter.g2.drone_show_manager.has_explicit_show_origin_set_by_user() &&
            copter.g2.drone_show_manager.has_explicit_show_origin_set_by_user()
        )
    );
}

// 
bool ModeDroneShow::use_pilot_yaw(void) const
{
    return copter.mode_guided.use_pilot_yaw();
}

//* 判断当前是否着陆
bool ModeDroneShow::is_landing() const
{       
    switch (_stage)
    {
        case DroneShow_Landing:
            return true;
        case DroneShow_RTL:
            return copter.mode_rtl.is_landing();
        default:
            return false; 
    }
    return false;
}

bool ModeDroneShow::is_taking_off() const
{
    //? 为什么要有个 reached_wp_destination ? 
    return ( (_stage == DroneShow_Takeoff) && !wp_nav->reached_wp_destination() );
}

//* 处理来自地面站的起飞请求。可用于表演开始之前的起飞测试
//! 主要用于起飞测试，不是表演开始！！！！ 
// 我们需要重写Mode::do_user_takeoff_start的默认实现，因为它需要使用油门起飞
// 这里我们只是将无人机发送到“起飞”状态。
bool ModeDroneShow::do_user_takeoff_start(float takeoff_alt_cm)
{
    // takeoff_alt_cm 被忽略了，这是经过考虑了的，不想让 takeoff_start() 的逻辑复杂
    // 起飞的高度可以在参数中设置 "SHOW_TAKEOFF_ALT"
    if( try_to_start_motors_if_prepared_to_take_off() ){
        takeoff_start();
    }

    // 如果没有定义表演的原点起飞可能失败。
    // 所以我们需要检测是否进入了 takeoff stage 同时返回 false 如果没有进入
    if( _stage == DroneShow_Takeoff ){
        // 在起飞后进入悬停模式，给地面站返回成功。
        _next_stage_after_takeoff = DroneShow_Loiter;
        return true;
    } else {
        // 给地面站返回失败
        return false;
    }

}

// 获取目标点
bool ModeDroneShow::get_wp(Location& destination) const
{
    switch(_stage) {
        case DroneShow_Performing:
            return copter.mode_guided.get_wp(destination);
        case DroneShow_Loiter: 
            return copter.mode_loiter.get_wp(destination);
        case DroneShow_Landing:
            return copter.mode_land.get_wp(destination);
        case DroneShow_RTL:
            return copter.mode_rtl.get_wp(destination);
        default:
            return false;
    }
}

// 距离目标点的水平距离（单位为厘米）。
uint32_t ModeDroneShow::wp_distance() const
{
    switch(_stage) {
        case DroneShow_Performing:
            return copter.mode_guided.wp_distance();
        case DroneShow_Landing:
            return copter.mode_land.wp_distance();
        case DroneShow_RTL:
            return copter.mode_rtl.wp_distance();
        default:
            return false;
    }
}

// 返回无人机到目标点的航向角（bearing）
int32_t ModeDroneShow::wp_bearing() const 
{
    switch(_stage) {
        case DroneShow_Performing:
            return copter.mode_guided.wp_bearing();
        case DroneShow_Landing:
            return copter.mode_land.wp_bearing();
        case DroneShow_RTL:
            return copter.mode_rtl.wp_bearing();
        default:
            return false;
    }
}

// 用于在不同表演阶段返回无人机当前的横向误差（crosstrack error）
float ModeDroneShow::crosstrack_error() const
{
    switch(_stage) {
        case DroneShow_Performing:
            return copter.mode_guided.crosstrack_error();
        case DroneShow_Loiter: 
            return copter.mode_loiter.crosstrack_error();
        case DroneShow_Landing:
            return copter.mode_land.crosstrack_error();
        case DroneShow_RTL:
            return copter.mode_rtl.crosstrack_error();
        default:
            return false;
    }
}

bool ModeDroneShow::cancel_requested() const 
{
    return copter.g2.drone_show_manager.cancel_requested();
}

int32_t ModeDroneShow::get_default_yaw_cd() const
{
    return copter.initial_armed_bearing;
}

// 函数用于计算自上次尝试重置“Home”位置以来经过的时间（以毫秒为单位）
//! 在 skybrush 中写的 int32_t 我觉得有问题
uint32_t ModeDroneShow::get_elapsed_time_since_last_home_position_reset_attempt_msec() const
{
    // AP_HAL::mills() 返回一个 uint32_t 类型的值，表示当前系统时间的毫秒数
    return AP_HAL::millis() - _last_home_position_reset_attempt_at;
}

uint32_t ModeDroneShow::get_elapsed_time_since_last_stage_change_msec() const
{
    return AP_HAL::millis() - _last_stage_change_at;
}

// 检测相关参数的变化，同时打印到 console
// 主要检测 授权状态，启动时间
void ModeDroneShow::check_change_in_parameters()
{
    // 上次检测到的授权状态
    static bool last_seen_authorization;
    // 上次检测到的启动时间
    static uint64_t last_seen_start_time;
    bool current_authorization = copter.g2.drone_show_manager.has_authorization_to_start();
    uint64_t current_start_time = copter.g2.drone_show_manager.get_start_time_epoch_undefined();

    if( last_seen_authorization != current_authorization ){
        last_seen_authorization = current_authorization;
        notify_start_time_changed();
    }

    if( last_seen_start_time !=  current_start_time  ){
        last_seen_start_time = current_start_time;
        notify_authorization_changed();
    }
}

void ModeDroneShow::notify_start_time_changed()
{
    // 清除飞行前验证标记，无论之前是否执行
    _preflight_calibration_done = false;

    // 清除 home position
    _home_position_set = false;
}

void ModeDroneShow::notify_authorization_changed()
{
    // 处于等待起飞阶段，同时已经授权起飞
    if(_stage == DroneShow_WaitForStartTime && copter.g2.drone_show_manager.has_authorization_to_start()){
        // 更新 home position, 重载 AGL 为 0
        try_to_update_home_position();
    }
}

//! 在表演的时候发送一个 guided mode 命令  !!!!!!!!!
//! performing_run 发一个请求，这个函数直接把读取和设置航点这件事都做了
//  计算飞行轨迹
bool ModeDroneShow::send_guided_mode_command_during_performance()
{
    AC_DroneShowManager::GuidedModeCommand command;

    // 返回的参数都在 command 里面
    if( copter.g2.drone_show_manager.get_current_guided_mode_command_to_send(
        command,get_default_yaw_cd(),
        _altitude_locked_above_takeoff_altitude
    )) {
        //! 使用 guided mode 的函数设置航点
        copter.mode_guided.set_destination_posvelaccel(
            command.pos, command.vel, command.acc,
            /* use_yaw = */ true, command.yaw_cd,
            /* use_yaw_rate =  */true, command.yaw_rate_cds
        );

        if( command.unlock_altitude ){
            _altitude_locked_above_takeoff_altitude = false;
        }

        copter.g2.drone_show_manager.notify_guided_mode_command_sent(command);

        return true;
    } else {
        return false;
    }
}

// 无论无人机是否准备好执行表演，如果电机尚未运行，则在表演前启动电机。
bool ModeDroneShow::start_motors_if_not_running()
{
    bool success = false;

    if( AP::arming().is_armed() ){
        // alreadly armed
        success = true;
    } else if( _prevent_arming_until_msec > AP_HAL::millis() ){
        // 拒绝 arming 因为近期已经试过了
    } else if( AP::arming().arm( AP_Arming::Method::SCRIPTING, 
               /* do_arming_checks = */ true))
    {
        success = true;
    } else {
        // 起飞前检测未通过，防止连续多次尝试
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
    }

    return success;
}

// 设置 home position 为无人机当前的位置
bool ModeDroneShow::try_to_update_home_position()
{
    _last_home_position_reset_attempt_at = AP_HAL::millis();

    if( !is_disarmed_or_landed() ){
        // 在飞行过程中不能设置
        return false;
    }
    return copter.set_home_to_current_location(/* lock = */ false);
}

// Starts the motors before the show if they are not running already, after
// checking whether the drone is prepared to take off (according to the
// show manager)
bool ModeDroneShow::try_to_start_motors_if_prepared_to_take_off()
{
    return copter.g2.drone_show_manager.is_prepared_to_take_off() && start_motors_if_not_running();
}

// 开始无人机初始化阶段
void ModeDroneShow::initialization_start()
{
    // 设置无人机的阶段
    _set_stage(DroneShow_Init);

    //? Assume normal operation: we will start performing after the takeoff
    _next_stage_after_takeoff = DroneShow_Performing;

    // 清除上次尝试解锁的时间的记录
    _prevent_arming_until_msec = 0;

    //* This is copied from ModeAuto::init()

    // initialise waypoint and spline controller
    //? what is spline controller? 
    wp_nav->wp_and_spline_init();

    // 清除 guided mode 的限制，我们会在内部使用 guided mode 来控制表演
    copter.mode_guided.limit_clear();

    // Set auto-yaw mode to HOLD
    // 不让无人机旋转
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    //* Part from ModeAuto::init() ends here

    // 清除所有依赖 start time 的状态信息
    notify_start_time_changed();

    // 通知 drone_show_manager , drone show mode 已初始化完成
    copter.g2.drone_show_manager.notify_drone_show_mode_initialized();
}

// 第一次激活的时候初始化 drone mode show
void ModeDroneShow::initialization_run()
{
    // 确定是否在空中
    if( is_disarmed_or_landed() ){
        // 不在空中的话就切换到 wait for start time 模式
        wait_for_start_time_run(); 
    } else {
        // 在空中的话就进入 loiter 模式，因为我们不知道 表演的时钟
        loiter_run();
    }
}

// 进入等待表演开始阶段
void ModeDroneShow::wait_for_start_time_start()
{
    _set_stage(DroneShow_WaitForStartTime);

    _motors_started = false;

    // 设置当前的位置为 home position
    try_to_update_home_position();
}

//!  wait for the start time run
//!  怎么没有检测磁罗盘是否校准那些
void ModeDroneShow::wait_for_start_time_run()
{
    float time_until_takeoff_sec = copter.g2.drone_show_manager.get_time_until_takeoff_sec();
    float time_since_takeoff_sec = - time_until_takeoff_sec;
    //? 5 秒之后就不起飞了
    const float latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds = 5.0f;

    // 所有参数都设置为 0 
    attitude_control->reset_yaw_target_and_rate();
    attitude_control->reset_rate_controller_I_terms();
    pos_control->standby_xyz_reset();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f,0.0f,0.0f);

    // This is copied from ModeStabilize::run()
    if( !motors->armed() ){ // 无人机没有解锁 -> 关闭电机
        motors -> set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if( !copter.ap.land_complete ){ // 没有在地面 -> 取消油门限制
        motors -> set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {//! 解锁了且在地面，进入怠速（这里可能要改）
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // 错过起飞了, 切换到 land 或者 loiter 模式
    if( time_since_takeoff_sec > latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds + 1 ){
        if( is_disarmed_or_landed() ){
            landing_start();
        } else {
            // 理论上，应该不存在这种情况
            loiter_start();
        }
    } else {
        // 在起飞的10 秒内
        if( time_until_takeoff_sec <= 10 ){
            // 检查气压计
            if( !_preflight_calibration_done ){
                // barometer 气压计
                // This is copied from GCS_MAVLINK::_handle_command_preflight_calibration_baro()
                AP::baro().update_calibration();

                _preflight_calibration_done = true;
            }
            
            // 设置 home position
            if( !_home_position_set ){
                if( !try_to_update_home_position() ){
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not set home position, giving up");
                    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
                    error_start();
                } else {
                    _home_position_set = true;
                }
            }
        } else { //! 还有很多时间，直接设为 false （maybe need change ）
            _preflight_calibration_done = false;
            _home_position_set = false;
        }

        //! 剩下的都是需要起飞授权的 ( 起飞授权这套逻辑不好，后面要改，这是给使用地面站的人增加负担 )
        if( copter.g2.drone_show_manager.has_authorization_to_start() ){
            if( time_until_takeoff_sec <=8 && !_motors_started ){
                // We attempt to start the motors 8 seconds before our takeoff time,
                // and we keep on doing so until 5 seconds after the takeoff time, when
                // we give up.
                //
                // No need to set the home position once again; arming the motors
                // will reset AGL to zero.
                if( time_since_takeoff_sec < latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds ){
                    if( try_to_start_motors_if_prepared_to_take_off() ){
                        _motors_started = true;
                    }
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start motors, giving up");
                    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
                    error_start();
                }
            }

            if( time_until_takeoff_sec <= 0 && _motors_started && _home_position_set ){
                //* time to take off!
                takeoff_start();
            }
        }else {
            // 如果没有授权，停止已开始的电机。电机有可能是在地面站测试旋转。
            if(_motors_started) {
                if(AP::arming().is_armed()){
                    AP::arming().disarm(AP_Arming::Method::SCRIPTING);
                }

                _motors_started = false;
            }
        }
    }
}

void ModeDroneShow::takeoff_start()
{
    Location current_loc(copter.current_loc);
    int32_t current_alt, target_alt;

    // 检测无人机是否知道自己的位置
    if( !copter.current_loc.initialised() ){
        // This should not happen, but nevertheless let's move to the
        // error state if we don't know where we are
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to take off, no known location");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
        error_start();
        return;
    }

    // 通知 drone_show_manager 要起飞了。 drone_show_manager 可能
    // 拒绝起飞，因为 show orign 或者 orientation 没有设置
    if( !copter.g2.drone_show_manager.notify_takeoff_attempt() ){
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff cancelled by show manager");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
        error_start();
        return;
    }

    // 获取当前 ekf 的估计值， auto_takeoff_start 需要这个参数
    if( !current_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, current_alt)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to get current altitude above home");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        error_start();
        return;
    }

    // 基本的检测结束，可以进入起飞模式了
    _set_stage(DroneShow_Takeoff);

    //! 设置起飞的目标值
    target_alt = current_alt + copter.g2.drone_show_manager.get_takeoff_altitude_cm();

    //* the body of this function from here on is mostly adapted from
    // ModeAuto::takeoff_start()

    // 清零 I 项可以防止在起飞时积累的误差影响起飞的平稳性，确保无人机从一个“干净”的状态开始。
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(target_alt, /* terrain_alt = */false);

    //* part adapted from ModeAuto::takeoff_start() ends here

    // 确保偏航角的目标值和我们当前的值一样，清零 I 项避免积累误差在起飞中产生不必要的扰动。
    attitude_control->reset_yaw_target_and_rate();
    attitude_control->reset_rate_controller_I_terms();

    // 设置偏航目标为无人机起飞时的初始方向
    auto_yaw.set_fixed_yaw(
        /* [cd] -> [deg] */ get_default_yaw_cd() * 0.01f, 
        /* turn_rate_dps = */ 0, /* direction = */ 0, /* relative_angle = */ 0
    );

    // 将无人机状态设置为自动起飞模式所需的“自动解锁”状态。
    copter.set_auto_armed(true);

    // 设置未着陆
    copter.set_land_complete(false);
}

//* 执行起飞
void ModeDroneShow::takeoff_run()
{
    bool completed = false;

    auto_takeoff.run();

    if( cancel_requested() ){
        // 如果取消了起飞，则立刻降落
        landing_start();
    } else if( !motors->armed()){
        // 如果电机还没解锁，那就是前面某个环节出问题了
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during takeoff");
        error_start();
    } else if( takeoff_completed() ){
        completed = true;
    }
    
    if( completed ) {
        // 切换到下一个阶段, 正常来说应该是进入 performing 
        switch(_next_stage_after_takeoff){
            case DroneShow_Loiter:
                loiter_start();
                break;
            case DroneShow_Landing:
            case DroneShow_Landed:
                landing_start();
                break;
            default:
                performing_start();
        }

        _next_stage_after_takeoff = DroneShow_Performing;
    }
}

//* 返回是否成功的起飞，只会在 takeoff stage run 中调用
bool ModeDroneShow::takeoff_completed() const
{
    if( _stage == DroneShow_Takeoff ) {
        if( _next_stage_after_takeoff == DroneShow_Performing ){
            // 下一步开始就会跟随舞步文件，只有在至少达到预定高度的 70%
            // 同时现在的位置高于起飞高度时才会进入下一个阶段
            Location loc;
            int32_t altitude_above_home_cm;
            int32_t desired_altitude_above_home_cm;
            AC_DroneShowManager* show_manager = &copter.g2.drone_show_manager;

            if(
                show_manager->get_current_location(loc) &&
                loc.get_alt_cm(Location::AltFrame::ABOVE_HOME,altitude_above_home_cm)
            ){
                //* 大于目标高度的 70%
                if( altitude_above_home_cm >= 0.7 * show_manager->get_takeoff_altitude_cm() )  {
                    //! 高度是够了，但是轨迹已经准备好了吗？ (need change)
                    float elapsed = show_manager->get_elapsed_time_since_start_sec();
                    show_manager->get_desired_global_position_at_seconds(elapsed,loc);

                    // 如果成功获取了说明航点已经准备好了
                    if( loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, desired_altitude_above_home_cm) ){
                        return desired_altitude_above_home_cm >= altitude_above_home_cm;
                    } else {
                        // This should not happen either, especially because we've already been
                        // through a successfull call to loc.get_alt_cm() if we managed to get
                        // here.
                        return false;
                    }
                } else {
                    // We are above 70% of the takeoff altitude but the trajectory
                    // is behind so wait until it catches up
                    return false;
                }
            } else {
                // This should not happen; it usually means that we do not have an
                // EKF origin yet. The safest is to return false so we do not
                // proceed to the "performing" phase with this error.
                return false;
            }
        } else {
            /* This branch belongs to the case when we will either start
             * loitering after takeoff, or we will land immediately. In both
             * cases, ensure that we spend at least ten seconds with taking off.
             * This is needed because wp_nav->reached_wp_destination() will
             * trigger as soon as we are within WPNAV_RADIUS of the target
             * altitude, and switching to loitering immediately will mean that
             * we start loitering at an altitude below the desired one.
             * Yes, this is an ugly hack, but there is no way to start the
             * loiter mode while also specifying a target altitude to loiter at
             */


            return wp_nav->reached_wp_destination() && takeoff_timed_out();
        }
    } else if (_stage >= DroneShow_Performing && _stage <= DroneShow_Landed) {
        return true;
    } else {
        return false;
    }
}

// 返回起飞是不是超时了
bool ModeDroneShow::takeoff_timed_out() const
{
    if (_stage == DroneShow_Takeoff) {
        if(get_elapsed_time_since_last_stage_change_msec() > 10000){
        }
        return get_elapsed_time_since_last_stage_change_msec() > 10000;
    } else {
        return false;
    }
}

//* 表演开始阶段
void ModeDroneShow::performing_start()
{
    _set_stage(DroneShow_Performing);

// 起落架
// #if AP_LANDINGGEAR_ENABLED
//     // optionally retract landing gear
//     copter.landinggear.retract_after_takeoff();
// #endif

    // guided mode 初始化
    copter.mode_guided.init(true);
    
    // 进入导航引导起始点时，初始化引导的起始时间和起始位置，供后续的导航限制检查参考
    copter.mode_guided.limit_init_time_and_pos();
}

//* 表演执行
void ModeDroneShow::performing_run()
{
    // 上次发送指令的时间
    static uint32_t last_guided_command = 0;
    // 是否退出了表演模式
    bool exited_mode = 0;
    // 当前的时间
    uint32_t now = AP_HAL::millis();
    // 获取预设的更新间隔
    uint32_t target_dt = copter.g2.drone_show_manager.get_controller_update_delta_msec();

    if( now - last_guided_command >= target_dt ) {
        //* send_guided_mode_command_during_performance 就把设置航点这件事做完了
        if( !send_guided_mode_command_during_performance() ){
            // Failed to send guided mode command; try to switch to position
            // hold instead. This should not happen anyway.
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to send guided mode command");
            loiter_start();
            exited_mode = 1;
        }
        //? 这里为什么不用 AP_HAL::millis() ?
        last_guided_command = now;
    }

    // 飞下一个航点
    if( !exited_mode ) {
        copter.mode_guided.run();
    }

    if( cancel_requested() ){
        rtl_start();
    } else if (!motors -> armed() ){
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during a show.
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during show");
        error_start();
    } else if( performing_completed() ){
        // 表演完了进入返回
        landing_start();
    }
}  

bool ModeDroneShow::performing_completed() const
{
    // TODO(ntamas): what if we are late and we are not at the designated landing
    // position yet?
    return copter.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}

void ModeDroneShow::landing_start()
{
    _set_stage(DroneShow_Landing);

    // TODO(ntamas): set stopping point of loiter nav properly so we land as
    // close to our destination as possible

    // call regular land flight mode initialisation and ask it to ignore checks
    copter.mode_land.init(/* ignore_checks = */ true);
}

void ModeDroneShow::landing_run()
{
    copter.mode_land.run();

    if( landing_completed() ){
        landed_start();
    }
}

bool ModeDroneShow::landing_completed() const
{
    if( _stage == DroneShow_Landing ) {
        return(
            copter.ap.land_complete && (
                motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE ||
                motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN
            )
        ); 
    } else {
        return false;
    }
}

//* 当表演出问题的时候用这个返回 home position
void ModeDroneShow::rtl_start()
{
    _set_stage(DroneShow_RTL);

    // call regular RTL flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(/* ignore_checks =  */ true);
}

void ModeDroneShow::rtl_run()
{
    copter.mode_rtl.run(/* disarm_on_land = */ false);

    if( rtl_completed() ){
        landed_start();
    }
}

bool ModeDroneShow::rtl_completed() const
{
    if( _stage == DroneShow_RTL ){
        return(
            copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
        );
    } else {
        return false;
    }
}

// starts the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_start()
{
    _set_stage(DroneShow_Loiter);

    // call regular position hold flight mode initialisation
    copter.mode_loiter.init(true);
}

// performs the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_run()
{
    // call regular position hold flight mode run function
    copter.mode_loiter.run();
}

// starts the phase where we have landed after a show and we do nothing any more
void ModeDroneShow::landed_start()
{
    _set_stage(DroneShow_Landed);

    copter.g2.drone_show_manager.notify_landed();
}

// performs the landed stage where we do nothing any more
void ModeDroneShow::landed_run()
{
    bool has_start_time = copter.g2.drone_show_manager.has_scheduled_start_time();

    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    //* 如果未来有开始的时间，直接进入下一个准备阶段
    // initialization process again
    if (has_start_time) {
        float time_until_start_sec = copter.g2.drone_show_manager.get_time_until_start_sec();

        // The upper limit (43200 sec = 12 hours) is needed to cater for the
        // case when the GCS sends us the _original_ start time of the show
        // again (after landing) as it will then be interpreted in the next
        // GPS week (since it is in the past in the current GPS week). We don't
        // want to go back to the "waiting for start time" state in this case.
        if (time_until_start_sec > 10.0f && time_until_start_sec <= 43200.f) {
            initialization_start();
        }
    }
}

// starts the error phase where we have failed to start a show and we do nothing any more
void ModeDroneShow::error_start()
{
    _set_stage(DroneShow_Error);
}

// performs the error stage where we do nothing any more
void ModeDroneShow::error_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// Sets the stage of the drone show module and synchronizes it with the DroneShowManager
void ModeDroneShow::_set_stage(DroneShowModeStage value)
{
    _stage = value;
    _last_stage_change_at = AP_HAL::millis();

    _altitude_locked_above_takeoff_altitude = (_stage == DroneShowModeStage::DroneShow_Performing);

    copter.g2.drone_show_manager.notify_drone_show_mode_entered_stage(_stage);
}
#endif

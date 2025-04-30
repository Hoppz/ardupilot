#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"

// 此函数的作用是取消预定的集体返航（Collective RTL）操作
// force 是即使在表演过程中，也会清除返航的操作
bool AC_DroneShowManager::clear_scheduled_collective_rtl(bool force)
{
    if (!force && get_stage_in_drone_show_mode() != DroneShow_Performing)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    _crtl_start_time_sec = 0;

    return true;
}

// 请求无人机表演管理器 安排集体返航操作（Return To Launch，RTL），
// 前提是表演正在进行，并且信号来自 遥控器
void AC_DroneShowManager::handle_rc_collective_rtl_switch()
{
    if (_are_rc_switches_blocked())
    {
        return;
    }

    schedule_collective_rtl_at_show_timestamp_msec(get_elapsed_time_since_start_msec());
}

// 根据表演的时间戳调度一个集体返回起飞（RTL，Return to Launch）操作
bool AC_DroneShowManager::schedule_collective_rtl_at_show_timestamp_msec(uint32_t timestamp_ms)
{
    if (get_stage_in_drone_show_mode() != DroneShow_Performing)
    {
        return false;
    }

    _crtl_start_time_sec = timestamp_ms / 1000.0f;

    return true;
}

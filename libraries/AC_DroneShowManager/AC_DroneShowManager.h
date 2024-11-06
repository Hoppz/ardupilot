#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/RGBLed.h>
#include <AP_Param/AP_Param.h>

#include <AC_HardFence/AC_HardFence.h>
#include <AC_WPNav/AC_WPNav.h>

// module/libskybrush
#include <skybrush/colors.h>

struct sb_trajectory_s;
struct sb_trajectory_player_s;

struct sb_light_program_s;
struct sb_light_player_s;

struct sb_yaw_control_s;
struct sb_yaw_player_s;

class DroneShowLEDFactory;
class DroneShowLED;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#  include <AP_HAL/utility/Socket.h>
#endif

//* 无人机表演处于的阶段
enum DroneShowModeStage {
    DroneShow_Off,                  // 未开启表演
    DroneShow_Init,                 // 表演初始化
    DroneShow_WaitForStartTime,     // 等待开始时间
    DroneShow_Takeoff,              // 起飞
    DroneShow_Performing,           // 表演
    DroneShow_RTL,                  // 返航
    DroneShow_Loiter,               // 悬停
    DroneShow_Landing,              // 下降中
    DroneShow_Landed,               // 已着陆
    DroneShow_Error,                // 错误
};

//* 无人机控制的枚举类
enum DroneShowControlModeFlag {
    DroneShowControl_VelocityControlEnabled = 1,        // 启用速度控制
    DroneShowControl_AccelerationControlEnabled = 2,    // 启用加速度控制
};

//* 表演前检测的枚举类：
// 用于在等待开始时间阶段（“waiting for start time”）时，
// 定期检查的一些额外的飞行前条件
// 如果无人机在准备阶段没有准备好就会进入这个阶段
enum DroneShowPreflightCheckFlag {
    DroneShowPreflightCheck_ShowNotConfiguredYet = (1 << 0),    // 飞行前配置未准备号
    // Flags from this point onwards are sent in the dedicated four bits of the
    // status packet
    DroneShowPreflightCheck_NotAtTakeoffPosition = (1 << 7),    // 未处于起飞状态
};

//* 地面站控制灯光的枚举类
enum LightEffectType {
    LightEffect_Off,                            // 灯光关闭
    LightEffect_Solid,                          // 灯光常亮
    LightEffect_Blinking,                       // 闪烁
    LightEffect_Breathing,                      // 呼吸灯
    LightEffect_Last = LightEffect_Breathing
};


//* 灯光的优先级
enum LightEffectPriority {
    LightEffectPriority_None = 0,       // 没有光效请求或没有指定优先级
    LightEffectPriority_Broadcast = 1,  // 表示来自地面控制站（GCS）的广播请求，通常是针对整个无人机群体（swarm）的光效控制。
    LightEffectPriority_Individual = 2, // 表示个体用户的请求，通常是针对某一特定设备或无人机的个别光效设置。
    LightEffectPriority_Internal = 3    // 表示内部请求，通常是来自设备本身的控制命令，这类请求的优先级最高
};

//* 这个枚举控制展示开始时如何进行时间同步，以确保所有设备在正确的时间启动。
enum TimeSyncMode {
    TimeSyncMode_Countdown = 0, // 忽略 SHOW_START_TIME (GPS 的时间) 并且依赖地面控制站（GCS）发送的倒计时消息来同步时间
    TimeSyncMode_GPS = 1        // 使用 SHOW_START_TIME 和基于 GPS 时间进行同步。
    // TimeSyncMode_GPS 这种方式通常用于需要精确时间同步的场景，例如无人机群体的精确协调，避免由于时间偏差而导致不必要的错位。
};

/// @class AC_DroneShowManager
/// @brief //*管理表演无人机的灯光以及轨迹
class AC_DroneShowManager{

private: 

    /// @brief //*管理和转换坐标数据，确保飞行表演在正确的坐标系统下进行
    class showCoordinateSystem{
    
    public: 
        // 表示展示坐标系统原点的纬度, 单位是 1e-7 度
        int32_t origin_lat;         

        // 表示展示坐标系统原点的经度，单位是 1e-7 度
        int32_t origin_lng;         

        // 表示展示坐标系统原点的海拔高度（相对于海平面）,单位是毫米
        int32_t origin_amsl_mm;     

        // 表示展示坐标系统的 X 轴的方向，单位是弧度
        // 这个值描述了坐标系相对于全球坐标系的旋转角度，帮助确定坐标系统的定向。
        float orientation_rad;

        // 一个布尔值，表示海拔高度（origin_amsl_mm）是否有效。
        // 如果为 true，则海拔高度是相对于海平面的（AMSL）
        // 如果为 false，则展示的坐标是相对于地面（AGL）
        bool origin_amsl_valid;

        // 清除展示坐标系统的状态。具体来说，将坐标原点重置为“空”（Null Island）
        // 即 (0, 0) 坐标。这会把坐标系统重置为无效状态。
        void clear();

        // 将展示坐标系中的坐标转换为全球 GPS 坐标系中的坐标
        // 并将转换后的全球坐标存储到 loc 中
        void convert_show_global_coordinate(sb_vector3_with_yaw_t vec, Location& loc) const;

        // 将展示坐标系中的航向角度 (yaw) 转换为相对于正北方向的百分之一度，并进行缩放。
        // 此方法用于将展示系统中的角度转换为全球标准的角度表示。
        void convert_show_to_global_yaw_and_scale_to_cd(float value) const;

        // 判断当前坐标系统是否有效。
        // 一个坐标系统被认为是有效的，当且仅当原点的纬度和经度不为零。
        bool is_vaild() const { return origin_lat != 0 && origin_lng != 0; };
    };

public:
    AC_DroneShowManager();
    ~AC_DroneShowManager();

    //! 不允许赋值定义
    AC_DroneShowManager(const AC_DroneShowManager &other) = delete;
    AC_DroneShowManager &operator=(const AC_DroneShowManager&) = delete;

    // 用于描述在无人机表演中当前设置的开始时间的来源
    enum StartTimeSource {
        NONE = 0,           // 没有设置开始时间
        PARAMATER = 1,      // 开始时间通过用户设置的参数 START_TIME 进行配置
        START_METHOD = 2,   // 开始时间是通过调用 schedule_delayed_start_after() 方法设置的
        RC_SWITCH = 3       // 开始时间是通过遥控开关设置的, 在特定的遥控状态下启动无人机表演
    };

    // 在表演时会被发送的 guided mode 的命令参数
    struct GuidedModeCommand {
        Vector3f pos;
        Vector3f vel;
        Vector3f acc;
        bool unlock_altitude;   // 高度是否被限制了
        float yaw_cd;           // _cd 表示 百分之一，如 1000 表示 10°
        float yaw_rate_cds;     // 偏航速率 500 , 表示 5°/s

        void clear() {
            pos.zero();
            vel.zero();
            acc.zero();
            unlock_altitude = false;
            yaw_cd = 0.0f;
            yaw_rate_cds = 0.0f;
        }

    };
    // 在启动过程中的早期阶段初始化系统，确保即使在内存有限的情况下（如Pixhawk1）
    // 也能为接下来的操作分配足够的内存。
    void early_init();

    // 在飞行表演时的初始化, 读入航点信息
    void init(const AC_WPNav* wp_nav);

    // 返回一个布尔值，表示用户是否要求尽快取消当前的飞行表演
    // 该变量通常在 mode_drone_show.cpp 检查
    void cancel_requested() const{ return _cancel_requested; }

    // 此函数的作用是取消预定的集体返航（Collective RTL）操作
    // force 是即使在表演过程中，也会清除返航的操作
    bool clear_scheduled_collective_rtl(bool force = false);

    // 清除预定的表演开始时间, 如果已经开始则不会取消
    bool clear_scheduled_start_time(bool force = false);

    //! 配置表演的坐标系统，包括起始位置、方向和海平面高度（AMSL）
    //! 配置整个系统的坐标原点
    bool configure_show_coordinate_system(
        int32_t lat, int32_t lon, int32_t amsl_mm, float orientation_deg
    ) WARN_IF_UNUSED;

    // 返回指定时间（表演开始后的秒数）时，RGB 灯光的颜色。
    void get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color);

    // 返回在表演执行过程中，连续两个 guided mode 控制命令之间的推荐时间间隔，单位为毫秒
    // 该函数用于获取控制器在表演过程中的更新频率。无人机会根据这个间隔周期性地接收新的控制命令。
    uint32_t get_controller_update_delta_msec() const { return _controller_update_delta_msec; }

    // 返回当前应该发送的 guided mode 控制命令
    // 传入一个 GuidedModeCommand 类型的引用，函数会填充这个对象，
    // 用于指定当前应该发送的控制命令
    bool get_current_guided_mode_command_to_send(
        GuidedModeCommand& command,
        int32_t default_yaw_cd,                             // 默认的航向
        bool altitude_locked_above_takeoff_altitude = false // 如果为 true，则锁定飞行高度在起飞高度之上。
    ) WARN_IF_UNUSED;

    // 获取飞行器的 当前绝对位置，即在 地理坐标系 中的位置（比如经纬度、海拔）。
    virtual bool get_current_location(Location& loc) const {return false;}

    // 获取飞行器相对于 EKF 原点 的 相对位置，坐标采用 NED（北东上）坐标系，单位是米（m）。
    virtual bool get_current_relative_position_NED_origin(Vector3f& vec) const {return false;}

    // 返回指定时间点（time）后，无人机在全球坐标系中的 期望位置（单位：厘米）。
    void get_desired_global_position_at_seconds(float time, Location& loc);

    // 返回 指定时间点（time）后，无人机在 全球 NEU 坐标系 中的 期望速度，单位是厘米每秒（cm/s）。
    void get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel);

    // 返回 指定时间点（time）后，无人机在 全球 NEU 坐标系 中的 期望加速度，单位是厘米每秒平方（cm/s²）。
    void get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(float time, Vector3f& acc);

    // 返回 指定时间点（time）后，无人机的 期望航向（Yaw），单位为 厘度（centidegrees）
    float get_desired_yaw_cd_at_seconds(float time);

    // 返回 指定时间点（time）后，无人机的 期望航向变化率（Yaw Rate），单位为 厘度每秒
    float get_desired_yaw_rate_cds_at_seconds(float time);

    // 返回飞行器当前位置与 期望目标位置 之间的 距离，并存储在 vec 中。
    void get_distance_from_desired_position(Vector3f& vec) const;

    //! 返回 无人机起飞时的位置，即表演开始前设定的起飞坐标。
    bool get_global_takeoff_position(Location& loc) const;

    // 返回无人机 RGB LED 灯最后发出的颜色，并存储在 color 中
    void get_last_rgb_led_color(sb_rgb_color_t& color) const { color = _last_rgb_led_color; }

    // 返回 相对于表演开始的着陆时间，单位为秒。该值表示从表演开始起，直到无人机着陆的时刻经过的时间。
    float get_relative_landing_time_sec() const { return _landing_time_sec; }

    // 返回 相对于表演开始的起飞时间，单位为秒。返回的值表示无人机从表演开始起，经过的时间
    float get_relative_takeoff_time_sec() const { return _takeoff_time_sec; }

    // 返回表演的 开始时间，以微秒为单位
    // 取决于 SHOW_SYNC_MODE 参数
    //! 只用确认开始时间是否被改变
    uint64_t get_start_time_epoch_undefined() const {
        return (
            _params.time_sync_mode == TimeSyncMode_Countdown
            ? _start_time_on_internal_clock_usec
            : _start_time_unix_usec
        );
    }
    
    // 返回 整个表演轨迹的总时长，单位为秒
    float get_total_duration_sec() const { return _total_duration_sec; }

    // 返回自表演开始以来 已过去的时间，单位为微秒
    int64_t get_elapsed_time_since_start_usec() const;

    // 返回自表演开始以来 经过的时间，单位为毫秒
    int32_t get_elapsed_time_since_start_msec() const;

    // 返回自表演开始以来 经过的时间，单位为秒
    float get_elapsed_time_since_start_sec() const;

    // 返回当前表演模式下 无人机所处的阶段
    DroneShowModeStage get_stage_in_drone_show_mode() const { return _stage_in_drone_show_mode; }

    // 返回 起飞时无人机的目标高度，单位为厘米
    int32_t get_takeoff_altitude_cm() const { return _params.takeoff_altitude_m * 100.0f; }

    // 返回 无人机起飞的速度，单位为米每秒。
    // 它通过 wp_nav 对象获取默认的起飞速度（以厘米为单位），并将其转换为米每秒
    float get_takeoff_speed_m_s() const {
        float result = _wp_nav ? _wp_nav->get_default_speed_up() / 100.0f : 0;
        if (result <= 0) {
            /* safety check */
            result = DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC;
        }
        return result;
    }

    // 返回 表演开始前还剩多少时间，单位为微秒
    int64_t get_time_until_start_usec() const;

    //! 返回 表演开始前还剩多少时间，单位为秒
    float get_time_until_start_sec() const;
    
    //! 返回 起飞前的剩余时间，单位为秒
    float get_time_until_takeoff_sec() const;

    // 返回 着陆前的剩余时间，单位为秒
    float get_time_until_landing_sec() const;

    // 返回 速度前馈增益系数，用于速度控制。该系数决定了
    // 在速度控制系统中如何结合当前速度的期望值与实际值，进行控制命令的调整。
    float get_velocity_feedforward_gain() const { return _params.velocity_feedforward_gain; }

    //! 处理 MAVLink 用户命令，该命令通过中央 MAVLink 处理器转发到无人机表演管理器。
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    //! 处理 MAVLink 消息，该消息通过中央 MAVLink 处理器转发到无人机表演管理器。
    bool handle_message(const mavlink_message_t& msg) WARN_IF_UNUSED;

    // 请求无人机表演管理器 尽快安排开始表演
    void handle_rc_start_switch();

    // 请求无人机表演管理器 安排集体返航操作（Return To Launch，RTL），前提是表演正在进行，并且信号来自 遥控器
    void handle_rc_collective_rtl_switch();

    // 是否获得了用户授权开始表演
    bool has_authorization_to_start() const;

    // 返回 用户是否显式设置了表演的高度
    bool has_explicit_show_altitude_set_by_user() const;

    // 返回 用户是否显式设置了表演的起始位置（原点）
    bool has_explicit_show_origin_set_by_user() const;

    // 返回 用户是否显式设置了表演的朝向
    bool has_explicit_show_orientation_set_by_user() const;

    // 检查是否已为表演设置了 预定开始时间。
    bool has_scheduled_start_time() const {
        return (
            uses_gps_time_for_show_start()
            ? _start_time_unix_usec > 0
            : _start_time_on_internal_clock_usec > 0
        );
    }

    // 检查是否已经确定了 有效的起飞时间
    bool has_valid_takeoff_time() const {
        return _takeoff_time_sec >= 0 && _landing_time_sec > _takeoff_time_sec;
    }

    //! 检查无人机是否属于指定的 分组。 (need change)
    bool is_in_group(uint8_t index) const {
        return _params.group_index == index;
    }

    //! 检查无人机是否已准备好起飞。
    bool is_prepared_to_take_off() const;

    // 检查是否启用了加速度控制。
    bool is_acceleration_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_AccelerationControlEnabled;
    }

    // 检查是否启用了速度控制。
    bool is_velocity_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_VelocityControlEnabled;
    }

    // 检查在启动阶段是否成功加载了 表演文件 数据
    bool loaded_show_data_successfully() const;

    // 检查在启动时是否成功加载了 航向控制 数据
    bool loaded_yaw_control_data_successfully() const;

    //! 检查无人机是否符合给定的 分组掩码。(need change)
    bool matches_group_mask(uint8_t mask) const {
        return mask == 0 || mask & (1 << _params.group_index);
    }

    // 通知表演管理器无人机表演模式已 初始化
    void notify_drone_show_mode_initialized();

    // 通知表演管理器无人机表演模式已经 退出
    void notify_drone_show_mode_exited();

    // 通知表演管理器无人机表演模式已 进入指定阶段
    void notify_drone_show_mode_entered_stage(DroneShowModeStage stage);

    // 通知表演管理器无人机已 发送引导模式命令。
    void notify_guided_mode_command_sent(const GuidedModeCommand& command);

    // 通知表演管理器无人机已经 着陆
    void notify_landed();

    // 通知表演管理器无人机即将 尝试起飞
    bool notify_takeoff_attempt() WARN_IF_UNUSED;


    // 处理 MAVLink 的 CMD_USER1 消息，根据 do_clear 参数的值，重新加载或清除表演。
    bool reload_or_clear_show(bool do_clear) WARN_IF_UNUSED;

    // 请求重新从存储中加载表演文件，成功返回 true
    bool reload_show_from_storage() WARN_IF_UNUSED;

    // 根据表演的时间戳调度一个集体返回起飞（RTL，Return to Launch）操作
    bool schedule_collective_rtl_at_show_timestamp_msec(uint32_t timestamp_ms);

    // 调度表演的延迟开始。延迟时间由 delay_ms 参数指定，单位为毫秒
    // 在 delay_ms 之后开始执行表演
    bool schedule_delayed_start_after(uint32_t delay_ms);

    // 通过指定的 MAVLink 通道发送表演状态信息
    void send_drone_show_status(const mavlink_channel_t chan) const;

    // 判断无人机是否应该在开机时自动切换到表演模式，如果没有遥控器输入。
    bool should_switch_to_show_mode_at_boot() const;

    // 判断当无人机获得启动授权后，是否应该切换到表演模式。
    bool should_switch_to_show_mode_when_authorized() const;

    // 如果当前表演正在运行，则请求表演管理器尽快取消表演
    void stop_if_running();

    // 更新无人机的 LED 状态，并执行定期任务（例如检查参数变化）
    // 该函数需要以 50Hz 的频率调用，其中大部分子程序以 25Hz 执行
    void update();

    // 判断是否使用 GPS 时间来确定表演的开始时间
    bool uses_gps_time_for_show_start() const { return _params.time_sync_mode == TimeSyncMode_GPS; }

    // 将与无人机表演管理器相关的日志消息写入日志
    void write_log_message() const;

    static const struct AP_Param::GroupInfo var_info[];

    AC_HardFence hard_fence;

    // 这个常量表示无人机起飞时的默认垂直速度。如果在表演导航中，WPNAV_SPEED_UP 参数无效，系统就会使用这个默认起飞速度。
    static constexpr float DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC = 1.0f;

    // 这个常量表示无人机进行降落时的目标高度（即，降落之前，飞行到 3 米的高度）
    // 在执行降落阶段时，系统会确保无人机下降到这个指定的高度，然后再开始正式降落
    static constexpr float LANDING_ALTITUDE_METERS = 3.0f;
    
private:

    struct {

        // 用户设置的 GPS 周时间（单位：秒）。只在 time_sync_mode == TimeSyncMode_GPS 时有效
        // 如果使用 GPS 时间同步模式，这个参数用于指定表演的起始时间
        AP_Int32 start_time_gps_sec;

        // 表演坐标系统的纬度，单位是 1e-7 度（即 10^-7 度）。由用户设置
        AP_Int32 origin_lat;

        // 表演坐标系统的经度，单位是 1e-7 度。由用户设置
        AP_Int32 origin_lng;

        // 表演坐标系统的海平面高度，单位是毫米（mm）。由用户设置
        AP_Int32 origin_amsl_mm;

        // 表演坐标系统的方向，单位是度。由用户设置
        AP_Float orientation_deg;

        // 这个参数告诉系统，用户是否已经授权无人机开始表演
        AP_Int8 authorized_to_start;

        // 如果设置为真，系统会在启动时自动进入表演模式
        AP_Int8 show_mode_settings;

        // 该参数设置起飞前状态指示灯的亮度
        AP_Int8 preflight_light_signal_brightness;

        // 通过位掩码配置控制算法的各个方面，比如是否启用加速度控制、是否启用速度控制等
        AP_Int16 control_mode_flags;

        //! 指定每秒发送的指导模式命令的频率（Hz）
        AP_Int8 control_rate_hz;
        
        //! (need change)
        //! 指定该无人机所属的分组索引，最多支持 8 个组（索引 0 到 7）
        AP_Int8 group_index;

        //! 起飞前，XY 平面内允许的最大位置误差，单位是米（m）
        AP_Float max_xy_placement_error_m;

        //! 表演过程中，XY 平面内允许的最大偏离量，单位是米（m）
        AP_Float max_xy_drift_during_show_m;

        //! 表演过程中，Z 轴（高度）方向允许的最大偏离量，单位是米（m）
        AP_Float max_z_drift_during_show_m;

        // 该参数用于调整无人机控制系统中的速度前馈增益，以提升速度控制性能
        AP_Float velocity_feedforward_gain;

        // 起飞的目标高度，单位是米（m）
        AP_Float takeoff_altitude_m;

        // 控制无人机如何处理时间同步，决定是通过 GPS 还是通过内部时钟来启动表演
        AP_Int8 time_sync_mode;

        // 该结构体用于定义 LED 灯光的配置
        struct {
            
            AP_Int8 type;                   // LED 类型

            AP_Int8 channel;                // LED 所在的通道

            AP_Int8 count;                  // ED 灯带上的灯泡数量（仅对 NeoPixel 或 ProfiLED 类型有效）。

            AP_Float gamma;                 // LED 灯的伽马修正指数
            
            AP_Float white_temperature;     // LED 的色温（适用于带有额外白光 LED 的灯光）
        } led_specs[1];
    } _params;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 这个套接字被用来将 RGB LED 的状态（例如颜色、亮度等）传输到外部可视化工具，
    // 通常是为了调试或演示目的。
    SocketAPM _sock_rgb;

    // 表示 RGB LED 套接字是否已成功打开。
    bool _sock_rgb_open;
#endif

    // 保存从存储器加载的整个显示文件的内存区域
    uint8_t* _show_data;

    struct sb_trajectory_s* _trajectory;                // 用于存储与无人机表演相关的轨迹数据
    struct sb_trajectory_player_s* _trajectory_player;  //
    bool _trajectory_valid;

    struct sb_light_program_s* _light_program;
    struct sb_light_player_s* _light_player;
    bool _light_program_valid;

    struct sb_yaw_control_s* _yaw_control;
    struct sb_yaw_player_s* _yaw_player;
    bool _yaw_control_valid;

    // 由_update_preflight_check_result() 周期性更新，
    // _preflight_check_failures 会记录表演模式的预飞检查结果，表示是否有任何检查失败。
    // 具体的失败项可以参考 DroneShowPreflightCheckFlag 枚举值
    uint8_t _preflight_check_failures;

    // 这是当前飞行时使用的坐标系统，它包含了与表演相关的坐标原点、方向等信息。
    // 该坐标系统在飞行过程中会不断更新，并用于无人机在表演中定位和执行轨迹。
    ShowCoordinateSystem _show_coordinate_system;

    // 这个坐标系统由用户设置，并周期性地从参数中更新。与 _show_coordinate_system 不同，
    // 它不在飞行中使用，而是在表演开始前作为暂定坐标系统使用，
    // 可能会在后期更新为实际的表演坐标系统。
    ShowCoordinateSystem _tentative_show_coordinate_system;

    // 这个变量用于记录表演启动时间是由哪个来源请求设置的
    StartTimeSource _start_time_requested_by;




};  

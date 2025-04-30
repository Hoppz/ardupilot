#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/RGBLed.h>
#include <AP_Param/AP_Param.h>
#include <AC_WPNav/AC_WPNav.h>

#include <AC_HardFence/AC_HardFence.h>

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
    DroneShow_Performing,           // 表演         -> mode_guided
    DroneShow_RTL,                  // 返航         -> mode_rtl
    DroneShow_Loiter,               // 悬停         -> mode_Loiter
    DroneShow_Landing,              // 下降中       -> mode_land
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
    class ShowCoordinateSystem{
    
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
        void convert_show_to_global_coordinate(sb_vector3_with_yaw_t vec, Location& loc) const;

        // 将展示坐标系中的航向角度 (yaw) 转换为相对于正北方向的百分之一度，并进行缩放。
        // 此方法用于将展示系统中的角度转换为全球标准的角度表示。
        float convert_show_to_global_yaw_and_scale_to_cd(float value) const;

        // 判断当前坐标系统是否有效。
        // 一个坐标系统被认为是有效的，当且仅当原点的纬度和经度不为零。
        bool is_valid() const { return origin_lat != 0 && origin_lng != 0; };
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
        PARAMETER = 1,      // 开始时间通过用户设置的参数 START_TIME 进行配置
        START_METHOD = 2,   // 开始时间是通过调用 schedule_delayed_start_after() 方法设置的
        RC_SWITCH = 3       // 开始时间是通过遥控开关设置的, 在特定的遥控状态下启动无人机表演
    };

    //! 在表演时会被发送的 guided mode 的命令参数
    //! 这个 command 用于设置下一个航点
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
    
    void early_init();

    void init(const AC_WPNav* wp_nav);

    // 返回一个布尔值，表示用户是否要求尽快取消当前的飞行表演
    // 该变量通常在 mode_drone_show.cpp 检查
    bool cancel_requested() const{ return _cancel_requested; }

    bool clear_scheduled_collective_rtl(bool force = false);

    bool clear_scheduled_start_time(bool force = false);

    bool configure_show_coordinate_system(
        int32_t lat, int32_t lng, int32_t amsl_mm, float orientation_deg
    ) WARN_IF_UNUSED;

    // 返回指定时间（表演开始后的秒数）时，RGB 灯光的颜色。
    void get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color);

    // 返回在表演执行过程中，连续两个 guided mode 控制命令之间的推荐时间间隔，单位为毫秒
    // 该函数用于获取控制器在表演过程中的更新频率。无人机会根据这个间隔周期性地接收新的控制命令。
    uint32_t get_controller_update_delta_msec() const { return _controller_update_delta_msec; }

    bool get_current_guided_mode_command_to_send(
        GuidedModeCommand& command,
        int32_t default_yaw_cd,                             
        bool altitude_locked_above_takeoff_altitude = false
    ) WARN_IF_UNUSED;

    // 获取飞行器的 当前绝对位置，即在 地理坐标系 中的位置（比如经纬度、海拔）。
    virtual bool get_current_location(Location& loc) const {return false;}

    // 获取飞行器相对于 EKF 原点 的 相对位置，坐标采用 NED（北东上）坐标系，单位是米（m）。
    virtual bool get_current_relative_position_NED_origin(Vector3f& vec) const {return false;}

    void get_desired_global_position_at_seconds(float time, Location& loc);

    void get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel);

    void get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(float time, Vector3f& acc);

    float get_desired_yaw_cd_at_seconds(float time);

    float get_desired_yaw_rate_cds_at_seconds(float time);

    void get_distance_from_desired_position(Vector3f& vec) const;

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

    int64_t get_elapsed_time_since_start_usec() const;

    int32_t get_elapsed_time_since_start_msec() const;

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

    int64_t get_time_until_start_usec() const;

    float get_time_until_start_sec() const;
    
    float get_time_until_takeoff_sec() const;

    float get_time_until_landing_sec() const;

    // 返回 速度前馈增益系数，用于速度控制。该系数决定了
    // 在速度控制系统中如何结合当前速度的期望值与实际值，进行控制命令的调整。
    float get_velocity_feedforward_gain() const { return _params.velocity_feedforward_gain; }

    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    bool handle_message(const mavlink_message_t& msg) WARN_IF_UNUSED;

    void handle_rc_start_switch();

    void handle_rc_collective_rtl_switch();

    bool has_authorization_to_start() const;

    bool has_explicit_show_altitude_set_by_user() const;

    bool has_explicit_show_origin_set_by_user() const;

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

    bool is_prepared_to_take_off() const;

    // 检查是否启用了加速度控制。
    bool is_acceleration_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_AccelerationControlEnabled;
    }

    // 检查是否启用了速度控制。
    bool is_velocity_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_VelocityControlEnabled;
    }

    bool loaded_show_data_successfully() const;

    bool loaded_yaw_control_data_successfully() const;

    //! 检查无人机是否符合给定的 分组掩码。(need change)
    bool matches_group_mask(uint8_t mask) const {
        return mask == 0 || mask & (1 << _params.group_index);
    }

    void notify_drone_show_mode_initialized();

    void notify_drone_show_mode_exited();

    void notify_drone_show_mode_entered_stage(DroneShowModeStage stage);

    void notify_guided_mode_command_sent(const GuidedModeCommand& command);

    void notify_landed();

    bool notify_takeoff_attempt() WARN_IF_UNUSED;


    bool reload_or_clear_show(bool do_clear) WARN_IF_UNUSED;

    bool reload_show_from_storage() WARN_IF_UNUSED;

    bool schedule_collective_rtl_at_show_timestamp_msec(uint32_t timestamp_ms);

    bool schedule_delayed_start_after(uint32_t delay_ms);

    
    void send_drone_show_status(const mavlink_channel_t chan) const;

    bool should_switch_to_show_mode_at_boot() const;

    bool should_switch_to_show_mode_when_authorized() const;

    void stop_if_running();

    void update();

    // 判断是否使用 GPS 时间来确定表演的开始时间
    bool uses_gps_time_for_show_start() const { return _params.time_sync_mode == TimeSyncMode_GPS; }

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

    // 保存从存储器 (storage) 加载的整个舞步文件
    uint8_t* _show_data;

    struct sb_trajectory_s* _trajectory;                // 用于存储与无人机表演相关的轨迹数据
    struct sb_trajectory_player_s* _trajectory_player;  //
    bool _trajectory_valid;

    struct sb_light_program_s* _light_program;
    struct sb_light_player_s* _light_player;
    bool _light_program_valid;                          // 表示当前的程序是一个可用的程序（非空）

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
    // _check_changes_in_parameters() 会把新的坐标系复制进去
    ShowCoordinateSystem _tentative_show_coordinate_system;

    // 这个变量用于记录表演启动时间是由哪个来源请求设置的
    StartTimeSource _start_time_requested_by;

    // 基于无人机内部时钟的表演起始时间，单位为微秒
    // 只有当时间同步模式设置为使用基于倒计时的方法时，才使用此变量
    uint64_t _start_time_on_internal_clock_usec;

    // 表演的启动时间，以微秒为单位，采用 UNIX 时间戳。如果没有设置则为 0
    uint64_t _start_time_unix_usec;

    //! 这是表演开始时无人机的起飞位置。起飞位置是以表演坐标系统为基准的本地坐标
    Vector3f _takeoff_position_mm;

    // 这是相对于表演开始的时间（以秒为单位），定义了无人机应该开始降落的时刻。
    float _takeoff_time_sec;

    // 降落时间，相对于表演开始时的时间（秒）
    float _landing_time_sec;

    // 这是相对于表演开始的时间（以秒为单位），指定了无人机应该开始执行 RTL 的时刻。
    // 如果未安排此轨迹，则值为零
    float _crtl_start_time_sec;

    struct {
        uint32_t started_at_msec;       // 灯光信号的开始时间，单位为毫秒
        uint16_t duration_msec;         // 灯光信号的持续时间，单位为毫秒
        uint8_t color[3];               // 灯光信号的颜色，采用 RGB 形式存储
        LightEffectType effect;         // 灯光效果的类型，表示灯光信号的动态效果，例如闪烁、呼吸灯 
        LightEffectPriority priority;   // 灯光信号的优先级
        uint16_t period_msec;           // 灯光信号的周期，单位为毫秒。适用于需要周期性变化的灯光效果，比如闪烁或脉冲
        uint16_t phase_msec;            // 灯光信号的相位，单位为毫秒。与周期相关，控制灯光效果的开始时刻。
        bool enhance_brightness;        // 是否通过使用白色 LED 来增强亮度
        bool sync_to_gps;              // 是否将灯光信号与 GPS 时间同步
    } _light_signal;

    // 当前的表演模式阶段
    DroneShowModeStage _stage_in_drone_show_mode;

    // 整个表演的总持续时间，单位为秒
    float _total_duration_sec;

    // 一个标志位，用于指示是否已经请求取消当前的表演
    // 这个值会被 mode_drone_show.cpp 定期检查
    bool _cancel_requested;

    //  表示执行飞行表演时，连续发出引导模式（Guided Mode）命令之间的首选时间间隔，单位为毫秒
    uint32_t _controller_update_delta_msec;

    // 是一个工厂对象，它负责创建 RGB LED 实例，供无人机表演管理器控制
    DroneShowLEDFactory* _rgb_led_factory;

    // 这个 LED 控制对象负责管理和执行具体的灯光指令，如颜色变化、闪烁效果等
    DroneShowLED* _rgb_led;

    // 存储最近一次发送到 RGB LED 的颜色
    sb_rgb_color_t _last_rgb_led_color;

    // 保存最近一次发送的引导模式命令
    GuidedModeCommand _last_setpoint;

    // 表示遥控器（RC）启动开关被阻塞的时间，单位为毫秒。
    // 如果该时间戳存在，说明遥控器的启动开关在此时间之前不能激活
    uint32_t _rc_switches_blocked_until;

    // 是无人机启动时 STAT_BOOTCNT 参数的副本，表示启动次数。
    //! 系统会定期将这个值的低两位发送在状态数据包中，
    //! 供地面控制站（GCS）检测无人机的重启情况。
    uint16_t _boot_count;

    // 指向航点导航模块（AC_WPNav）的引用。通过该引用，表演管理器可以查询起飞相关的导航参数
    const AC_WPNav* _wp_nav;

    bool _are_rc_switches_blocked();

    void _check_changes_in_parameters();

    void _check_events();

    void _check_radio_failsafe();

    void _clear_start_time_after_landing();

    void _clear_start_time_if_set_by_switch();

    bool _copy_show_coordinate_system_from_parameters_to(
        ShowCoordinateSystem& coordinate_system
    ) const;

    // 在操作失败后触发灯光信号
    void _flash_leds_after_failure();

    // 在操作成功后触发灯光信号
    void _flash_leds_after_success();

    // 来触发一个灯光信号，目的是吸引注意力，通常由地面控制站（GCS）操作员触发
    void _flash_leds_to_attract_attention(LightEffectPriority priority);
    
    //! 控制无人机的 LED 灯闪烁，使用指定的颜色（通过 RGB 值）和指定的闪烁次数（count）。
    // priority 参数表示闪烁信号的优先级，而 enhance_brightness（默认值为 false）
    // 用于决定是否通过使用额外的白色 LED 来增强亮度
    void _flash_leds_with_color(
        uint8_t red, uint8_t green, uint8_t blue, uint8_t count,
        LightEffectPriority priority, bool enhance_brightness = false
    );

    // 回一个用于实现灯光信号的时间戳。
    // 当无人机有稳定的 GPS 信号时，该时间戳会与 GPS 秒同步
    uint32_t _get_gps_synced_timestamp_in_millis_for_lights() const;

    bool _handle_custom_data_message(uint8_t type, void* data, uint8_t length);

    bool _handle_data16_message(const mavlink_message_t& msg);

    bool _handle_data32_message(const mavlink_message_t& msg);
    
    bool _handle_data64_message(const mavlink_message_t& msg);

    bool _handle_data96_message(const mavlink_message_t& msg);

    // 处理来自地面站的 LED_CONTROL 类型的 MAVLink 消息
    bool _handle_led_control_message(const mavlink_message_t& msg);

    bool _is_at_expected_position() const;

    bool _is_at_takeoff_position() const;

    bool _is_close_to_position(const Location& target_loc, float xy_threshold, float z_threshold) const;

    bool _is_gps_time_ok() const;

    void _recalculate_trajectory_properties();

    // 用于请求无人机切换到无人机表演模式
    virtual void _request_switch_to_show_mode() {};

    bool _load_show_file_from_storage();
    void _set_light_program_and_take_ownership(struct sb_light_program_s *value);
    void _set_trajectory_and_take_ownership(struct sb_trajectory_s *value);
    void _set_show_data_and_take_ownership(uint8_t *value);
    void _set_yaw_control_and_take_ownership(struct sb_yaw_control_s *value);

    // 更新无人机上的 LED 灯状态。确保 LED 灯的显示与无人机当前的状态一致。
    // 必须 定期 调用，调用频率为 25 Hz，即每秒更新 25 次。
    void _update_lights();

    
    void _update_preflight_check_result(bool force = 0);

    // 更新无人机控制的 RGB LED 实例。更新是基于当前舵机或控制参数来改变 LED 配置
    // 这个函数不直接控制 LED 的状态，而是更新配置，以便与其他系统同
    void _update_rgb_led_instance();

    // 重复上一次发送的 RGB LED 控制命令。当控制信道不稳定时，用来保证 LED 状态的一致性
    void _repeat_last_rgb_led_command();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool _open_rgb_led_socket();
#endif

};  

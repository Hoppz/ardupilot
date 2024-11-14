#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class DroneShowLED
{
private:
    // 当前的 gamma 校正指数
    //! 用于调整显示的亮度和对比度。
    //! gamma correction 是用于矫正功率对颜色的影响
    // https://zhuanlan.zhihu.com/p/33637724
    float _gamma;

    // 一个包含 256 个元素的查找表，存储了不同亮度下的颜色值，供 gamma 校正使用
    uint8_t _gamma_lookup_table[256];

    // 最近一次设置的 RGB 
    uint8_t _last_red, _last_green, _last_blue, _last_white;  
    
    // LED 设置命令重复次数
    uint8_t _repeat_count; 

    // 我们仍然需要重复最后一个LED命令的次数。
    uint8_t _repeat_count_left;
    
public:
    DroneShowLED() :
        _gamma(0.0f), _last_red(0), _last_green(0), _last_blue(0), _last_white(0),
        _repeat_count(0), _repeat_count_left(0)
    {
        set_gamma(1.0f);
        set_repeat_count(1);
    };
    virtual ~DroneShowLED() {};

    // 初始化 LED
    virtual bool init() { return true; };

    // 设置 LED 的 gamma 校正指数。
    void set_gamma(float value) {
        if (is_equal(value, _gamma)) {
            return;
        }

        _gamma = value;

        _update_gamma_lookup_table();
        _reset_repeat_count();
    }

    // 设置 LED 命令的重复次数，即设置每个颜色命令需要重复执行的次数
    void set_repeat_count(uint8_t value) {
        if (value < 1) {
            value = 1;
        }

        if (value != _repeat_count) {
            _repeat_count = value;
            _reset_repeat_count();
        }
    }

    // 设置 LED 的 rgb 颜色，白色自动设置为 0
    // 
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
        set_rgbw(red, green, blue, 0);
    }

    // 
    void set_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
        if (red != _last_red || green != _last_green || blue != _last_blue || white != _last_white) {
            _last_red = red;
            _last_green = green;
            _last_blue = blue;
            _last_white = white;
            _reset_repeat_count();
        }

        repeat_last_command_if_needed();
    }

    // 检查 LED 是否支持白色通道
    virtual bool supports_white_channel() { return false; }

    // 如果需要，重复上一次的 RGB LED 设置命令
    void repeat_last_command_if_needed() {
        if (_repeat_count_left == 0) {
            return;
        }

        if (set_raw_rgbw(
            _gamma_lookup_table[_last_red],
            _gamma_lookup_table[_last_green],
            _gamma_lookup_table[_last_blue],
            _gamma_lookup_table[_last_white]
        )) {
            _repeat_count_left--;
        }
    }
protected:

    // 该方法设置 LED 的 原始颜色（即 RGBW 空间的颜色）
    // 传入的参数 red, green, blue, 和 white 是经过 gamma 校正后的值。
    virtual bool set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) = 0;

private:

    void _reset_repeat_count() {
        _repeat_count_left = _repeat_count;
    }

    // 更新 gamma 校正查找表
    // 它会根据当前的 gamma 校正值（_gamma）计算并更新一个查找表
    // 之后在设置 LED 颜色时可以直接使用这个查找表来进行 gamma 校正
    void _update_gamma_lookup_table();
};

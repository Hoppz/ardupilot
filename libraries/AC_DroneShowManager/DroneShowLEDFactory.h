#pragma once

#include "DroneShowLED.h"

// 支持的 LED 类型
enum DroneShowLEDType {

    // 不使用 LED 输出
    DroneShowLEDType_None = 0,

    // 用于在 mavlink DEBUG_VECT 中输出 LED 的信息
    DroneShowLEDType_MAVLink = 1,

    // LED 输出发送到 NeoPixel LED 灯带
    DroneShowLEDType_NeoPixel = 2,

    // LED 输出发送到 ProfiLED LED 灯带
    DroneShowLEDType_ProfiLED = 3,

    // Debug 输出，LED 颜色用 Mavlink STATUSTEXT 消息输出
    DroneShowLEDType_Debug = 4,

    // 模拟的 LED
    DroneShowLEDType_SITL = 5,

    // 连接在 servo 的 LED
    DroneShowLEDType_Servo = 6,

    // I2C 的 LED
    DroneShowLEDType_I2C = 7,

    // LED 输出与伺服通道关联，但具有反向极性
    DroneShowLEDType_InvertedServo = 8,

    // WGDrones LED
    DroneShowLEDType_WGDrones = 9,

    // LED 输出发送到 NeoPixel RGBW LED 灯带
    DroneShowLEDType_NeoPixel_RGBW = 10,

    // 四个字节（RGBW）的 I2C
    DroneShowLEDType_I2C_RGBW = 11,

    // LED 由 ArduPilot 的 AP_Notify 框架控制
    DroneShowLEDType_Notify = 12,
};


// 根据指定的 LED 类型创建相应的 LED 实例
class DroneShowLEDFactory
{

public:
    DroneShowLEDFactory();

    DroneShowLEDFactory(const DroneShowLEDFactory &other) = delete;
    DroneShowLEDFactory &operator=(const DroneShowLEDFactory&) = delete;

    DroneShowLED* new_rgb_led_by_type(
        DroneShowLEDType type, uint8_t channel, uint8_t num_leds
    );

};

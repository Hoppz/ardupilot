#pragma once

/// @file   AC_DroneShowLED_MAVLink.h
/// @brief  Drone show LED that sends its output via MAVLink messages to an external component.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"
#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * 将 RGB LED 的状态通过 MAVLink 消息发送到外部组件。
 * 与前面的调试版不同，这个类的输出通过 DEBUG_VECT 消息发送，而不是 STATUSTEXT 消息。
 * 它可以将 LED 的状态发送到任何支持 MAVLink 的外部设备或地面控制站。
 */
class DroneShowLED_MAVLink : public DroneShowLED {
public:
    DroneShowLED_MAVLink(uint8_t instance = 0);

    /* Do not allow copies */
    DroneShowLED_MAVLink(const DroneShowLED_MAVLink &other) = delete;
    DroneShowLED_MAVLink &operator=(const DroneShowLED_MAVLink&) = delete;

protected:
    bool init(void) override;
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;

private:
    mavlink_channel_t _chan;
    uint8_t _instance;
};

#pragma once

/// @file   AC_DroneShowLED_Debug.h
/// @brief  Drone show LED that sends its output in MAVLink STATUSTEXT messages for debugging purposes.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"
#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * 整个函数的输出会被转发到 mavlink 中
 */
class DroneShowLED_Debug : public DroneShowLED {
public:
    DroneShowLED_Debug() : DroneShowLED() {};

protected:
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;
    bool supports_white_channel() override { return true; }

private:
    uint8_t packet_overhead() const;
};

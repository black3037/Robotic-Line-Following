#ifndef ROBOT_SETTINGS_H_INCLUDED
#define ROBOT_SETTINGS_H_INCLUDED

#include <cstdint>

// See *.cpp file for documentation.
extern const int32_t FIRMWARE_VERSION;
extern const float WHEEL_RADIUS;
extern const float WHEEL_BASE;
extern const float GEAR_RATIO;
extern const float DIST_PER_COUNT;
extern const float ENCODER_2_DIST;
extern const float ENCODER_2_THETA;
extern const float MOTOR_SIGNS[2];
extern const float ENCODER_SCALES[2];
extern const float K_DEFAULT[4];
extern const float ACCEL_SCALES[3];
extern const float ACCEL_OFFSETS[3];
extern const float BATTERY_SCALE;
extern const float BATTERY_OFFSET;
extern const float FULL_BATTERY_VOLTAGE;
extern const float LOW_BATTERY_VOLTAGE;
extern const float CRITICAL_BATTERY_VOLTAGE;

#endif

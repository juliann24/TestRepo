#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

/**
 * @file
 *
 * @defgroup accelerometer accelerometer
 * @{
 * @ingroup accelerometer
 * @brief Accelerometer module.
 *
 * This file contains the code for handling the accelerometer.
 */

#include <stdint.h>

#include "mcutask.h"

#include "bma4_defs.h"

/// Defines the available modes
typedef enum {
    ACCEL_RANGE_2G = 2, ///< +/- 2G range.
    ACCEL_RANGE_4G = 4, ///< +/- 4G range.
    ACCEL_RANGE_8G = 8, ///< +/- 8G range.
    ACCEL_RANGE_16G = 16 ///< +/- 16G range.
} accel_ranges;

/// Defines the available filter bandwiths.
typedef enum {
    ACCEL_BNDW_8 = 8,
    ACCEL_BNDW_16 = 16,
    ACCEL_BNDW_31 = 31,
    ACCEL_BNDW_63 = 63,
    ACCEL_BNDW_125 = 125,
    ACCEL_BNDW_250 = 250,
    ACCEL_BNDW_500 = 500,
    ACCEL_BNDW_1000 = 1000,
} accel_filter_bandwiths;

/// Consecutive data points for slope or slow movement detection.
typedef enum {
    ACCEL_CONS_DATA_POINTS_1 = 1,
    ACCEL_CONS_DATA_POINTS_2 = 2,
    ACCEL_CONS_DATA_POINTS_3 = 3,
    ACCEL_CONS_DATA_POINTS_4 = 4,
} accel_consec_data_points;

typedef enum {
    ACCEL_ENABLE_X_AXIS = 1,
    ACCEL_ENABLE_Y_AXIS = 2,
    ACCEL_ENABLE_XY_AXIS = 3,
    ACCEL_ENABLE_Z_AXIS = 4,
    ACCEL_ENABLE_XZ_AXIS = 5,
    ACCEL_ENABLE_YZ_AXIS = 6,
    ACCEL_ENABLE_XYZ_AXIS = 7,
} accel_axis_enabled;


uint32_t accelerometer_init(struct bma4_dev* bma4);
void accelerometer_set_signals(task motion_detected_signal, task high_G_signal);
void accelerometer_reload_configs(uint8_t thres_high_G, uint8_t thres_slope);
uint8_t accelerometer_get_alarm_movement();
void  accelerometer_reset_alarm_movement();
int16_t accelerometer_get_temperature();
int16_t accelerometer_get_current_temperature();
int16_t accelerometer_get_acceleration_axis_x();
int16_t accelerometer_get_acceleration_axis_y();
int16_t accelerometer_get_acceleration_axis_z();
uint32_t accelerometer_enable_interrupts();
uint32_t accelerometer_disable_interrupts();
uint32_t  set_accelerometer_mode_travel();
uint32_t set_accelerometer_mode_low_power1();
uint32_t set_accelerometer_mode_normal();
uint32_t set_accelerometer_threshold_movement();


#endif // ACCELEROMETER_H

/**
 * @}
 */

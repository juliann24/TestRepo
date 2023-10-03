/**
 * @file
 *
 * @{
 * @ingroup accelerometer
 */
#include <nrf_error.h>
#include <nrf_gpio.h>
#include <nrf_delay.h>
#include <app_gpiote.h>
#include <app_timer.h>
#include <nrf_error.h>
#include <sdk_errors.h>

#include "bma4_defs.h"
#include "bma4.h"
#include "bma2x2.h"
#include "nrf_drv_config.h"


#include "accelerometer.h"
#include "settings.h"
#include "i2c.h"

/// GPIOTE user id for this module.
//static app_gpiote_user_id_t accel_gpiote_id;

/// Task to be called when the slope interruption happens.
static task motion_detected_task;

/// Task to be called when the high G interruption happens.
static task high_G_task;


/// Polling time to check if we are moving.
#define SLOPE_POLLING_TIME APP_TIMER_TICKS(2000, TIMER_PRESCALER)


/// Timer to poll the slope interruption counter to see if we are moving.
APP_TIMER_DEF(slope_polling_timer);

/// Slope interruption counter.
static volatile uint32_t slope_int_arrived;

/// Flag to check if we are already looking for movement.
static volatile uint8_t detecting_movement;

/*
 * Agregado para detectar evento de recolección
 * */

///Event type of alarm movement.
static uint8_t alarm_movement; //Alarm movement 0=OK, 1=garbage collect, 2=crash

/*
 * end
 * */


/// Struct to configure the BMA2X2's parameters and functions.
//static struct bma2x2_t bma2x2;
//static struct bma4_dev bma4 = { 0 };
/**
 * @brief bma2x2_get_reset_int Checks interrupts currently active and resets all flags.
 * @param int_flags Interrupt flag status.
 */
static void bma2x2_get_reset_int(uint8_t *int_flags)
{
    int32_t retval = ERROR;

    retval = bma2x2_get_intr_stat(int_flags);
    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);

    // Clear interrupt flags.
    retval += bma2x2_rst_intr(1);
    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);
}

static void check_rw_range(uint8_t reg_add, uint8_t reg_len)
{
    if(reg_add > 0x7E)
        APP_ERROR_CHECK_BOOL(0);

    if(reg_add + reg_len - 1 > 0x7E)
        APP_ERROR_CHECK_BOOL(0);
}


static void check_rw_range2x2(uint8_t reg_add, uint8_t reg_len)
{
    if(reg_add > 0x3F)
        APP_ERROR_CHECK_BOOL(0);

    if(reg_add + reg_len - 1 > 0x3F)
        APP_ERROR_CHECK_BOOL(0);
}

static int8_t bma_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    ret_code_t retval;

    check_rw_range(reg_addr, len);

    uint8_t dev_add = BMA4_I2C_ADDR_PRIMARY;

    for (int i=0; i< len; i++)
    {
        retval = i2c_tx(dev_add, &reg_addr, read_data+i, 1);

        APP_ERROR_CHECK(retval);

        reg_addr = (reg_addr+1) & 0x7E;
    }

    return 0;
}

/**
 * @brief bma_write Writes registers to the BMA250E accelerometer.
 *
 * @param dev_add Device address to access.
 * @param reg_add Base register address to write into (0x00 to 0x3F).
 * @param reg_data Values to write into registers.
 * @param reg_len Number of register to be written.
 *
 * @returns 0, kept for compatibility.
 */
static int8_t bma2x2_write(uint8_t dev_add, uint8_t reg_add, uint8_t * reg_data, uint8_t reg_len)
{
    ret_code_t retval;

    check_rw_range(reg_add, reg_len);

    for (int i=0; i< reg_len; i++)
    {
        retval = i2c_tx(dev_add, &reg_add, reg_data+i, 1);

        APP_ERROR_CHECK(retval);

        reg_add = (reg_add+1) & 0x3F;
    }

    return 0;
}



static int8_t bma_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    check_rw_range(reg_addr, len);

    uint8_t dev_add = 0x20;

    ret_code_t retval = i2c_rx(dev_add, &reg_addr, read_data, len);
    APP_ERROR_CHECK(retval);

    return 0;
}

/**
 * @brief bma_read Reads registers from the BMA250E accelerometer
 * @param dev_add Device address to access
 * @param reg_add Base register address to read (0x00 to 0x3F)
 * @param reg_data Holds read register values
 * @param reg_len number of registers to be read
 *
 * @returns 0, kept for compatibility.
 */
static int8_t bma2x2_read(uint8_t dev_add, uint8_t reg_add, uint8_t * reg_data, uint8_t reg_len)
{
    check_rw_range(reg_add, reg_len);

    ret_code_t retval = i2c_rx(dev_add, &reg_add, reg_data, reg_len);
    APP_ERROR_CHECK(retval);

    return 0;
}

static int8_t bma_burst_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    return(bma_read(reg_addr, read_data, len, &intf_ptr));
}

/**
 * @brief bma_burst_read Same as bma_read, but with uint32_t reg_len argument, used for
 * compatibility with the BMA API.
 * @param dev_add Device address to access
 * @param reg_add Base register address to read (0x00 to 0x3F)
 * @param reg_data Holds read register values
 * @param reg_len number of registers to be read
 *
 * @returns 0, kept for compatibility.
 */

//static int8_t bma2x2_burst_read(uint8_t dev_add, uint8_t reg_add, uint8_t * reg_data, u32 reg_len)
//{
//    return(bma2x2_read(dev_add, reg_add, reg_data, (uint8_t)reg_len));
//}

void accelerometer_delay_us(uint32_t period, void *intf_ptr){
    nrf_delay_us(period);
}

/**
 * @brief get_accel Gets the acceleration on each axis and the temperature.
 * @param bma_xyzt_data Struct to be filled with the data.
 * @return Status of the communication?
 */
int32_t get_accel(struct bma2x2_accel_data_temp * bma_xyzt_data)
{
    int32_t retval = ERROR;

    /*
     * Accessing the bma2x2acc_data_temp parameter by using sample_xyzt.
     * Read the accel XYZT data.
     */
    retval = bma2x2_read_accel_xyzt(bma_xyzt_data);

    return(retval);
}

static void check_if_we_are_moving()
{
    static uint8_t counter = 0;
    uint32_t retval;

    if(slope_int_arrived == 0)
    {
        counter = 0;
        detecting_movement = 0;
        retval = app_timer_stop(slope_polling_timer);
        APP_ERROR_CHECK(retval);
        return;
    }

    // At this point slope_int_arrived > 0
    slope_int_arrived = 0;
    counter++;

    if(counter >= settings_get_num_moving_samples())
    {
        counter = 0;
        detecting_movement = 0;
        (motion_detected_task)(NULL);

        alarm_movement=1;

        return;
    }

    // We still need to keep checking.
    detecting_movement = 1;
    app_timer_start(slope_polling_timer, SLOPE_POLLING_TIME, NULL);

}



static void interrupt_event_handler()
{

    if (nrf_gpio_pin_read(BMA_INT1_PIN)!= 0)
    {
        (high_G_task)(NULL);
        alarm_movement=2;

    }
    else if (nrf_gpio_pin_read(BMA_INT2_PIN)!= 0)
    {

        slope_int_arrived++;

        if(!detecting_movement)
        {
            detecting_movement = 1;
            check_if_we_are_moving();
        }
    }

}

static void set_slope_interruption_parameters(uint8_t slope_threshold)

{
    uint32_t retval;
    const modes_parameters m_p = settings_current_modes_parameters();

    // Slope detection threshold.
    retval = bma2x2_set_thres(BMA2x2_ACCEL_SLOPE_THRES, slope_threshold);

    // Consecutive data points.
    retval += bma2x2_set_durn(BMA2x2_ACCEL_SLOPE_DURN, m_p.slope_consec_points - 1);

    // Enable axis
    if(m_p.slope_enabled_axis & ACCEL_ENABLE_X_AXIS)
        retval += bma2x2_set_intr_enable(SLOPE_X_INTR, INTR_ENABLE);

    if(m_p.slope_enabled_axis & ACCEL_ENABLE_Y_AXIS)
        retval += bma2x2_set_intr_enable(SLOPE_Y_INTR, INTR_ENABLE);

    if(m_p.slope_enabled_axis & ACCEL_ENABLE_Z_AXIS)
        retval += bma2x2_set_intr_enable(SLOPE_Z_INTR, INTR_ENABLE);

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);
}

static void set_no_motion_interruption_parameters()
{
    uint32_t retval;
    const modes_parameters m_p = settings_current_modes_parameters();

    // Slow motion threshold.
    retval = bma2x2_set_thres(BMA2x2_ACCEL_SLOW_NO_MOTION_THRES, m_p.no_motion_thres);

    // Duration in seconds to let the no motion detection work.
    retval += bma2x2_set_durn(BMA2x2_ACCEL_SLOW_NO_MOTION_DURN, m_p.no_motion_duration - 1);

    if(m_p.no_motion_enabled_axis & ACCEL_ENABLE_X_AXIS)
        retval += bma2x2_set_slow_no_motion(BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_X, INTR_ENABLE);

    if(m_p.no_motion_enabled_axis & ACCEL_ENABLE_Y_AXIS)
        retval += bma2x2_set_slow_no_motion(BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Y, INTR_ENABLE);

    if(m_p.no_motion_enabled_axis & ACCEL_ENABLE_Z_AXIS)
        retval += bma2x2_set_slow_no_motion(BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Z, INTR_ENABLE);

    // Enable no motion detection.
    retval += bma2x2_set_slow_no_motion(BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SEL, INTR_ENABLE);

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);
}


static void set_high_g_interruption_parameters(uint8_t high_G_threshold){
    uint32_t retval;
    const modes_parameters m_p = settings_current_modes_parameters();

    // HIGH G threshold for default is 0xFF is the more hight
    retval = bma2x2_set_thres(BMA2x2_ACCEL_HIGH_THRES, high_G_threshold);

    // Duration in seconds to let the high g detection work.
    retval += bma2x2_set_durn(BMA2x2_ACCEL_HIGH_DURN, m_p.high_G_duration - 1);
    retval += bma2x2_set_intr_enable(BMA2x2_HIGH_G_X_INTR, INTR_ENABLE);
    retval += bma2x2_set_intr_enable(BMA2x2_HIGH_G_Y_INTR, INTR_ENABLE);
    retval += bma2x2_set_intr_enable(BMA2x2_HIGH_G_Z_INTR, INTR_ENABLE);

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);


}

static void set_orientation_interruption_parameters()
{
    uint32_t retval;
    //const modes_parameters m_p = settings_current_modes_parameters();

    retval = bma2x2_set_orient_mode(0x02);
    //orientation blocking. generation of the interrupt can be blocked acording to conditions table 17  pag 37 datasheet.
    retval += bma2x2_set_orient_block(0x03);
    //set histerisis 1LSB always corresponds to 62.5mg increment is independent from g-range setting.
    retval +=bma2x2_set_orient_hyst(0x5);
    // orientation changes respect to the z_axis are ignored.
    retval += bma2x2_set_orient_enable(0x01);

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);
}

static void set_accelerometer_main_parameters()
{
    uint32_t retval;
    const modes_parameters m_p = settings_current_modes_parameters();

    // Set bandwidth.
    switch(m_p.accel_filter_bndw) {
    case ACCEL_BNDW_8:
        retval = bma2x2_set_bw(BMA2x2_BW_7_81HZ);
        break;
    case ACCEL_BNDW_16:
        retval = bma2x2_set_bw(BMA2x2_BW_15_63HZ);
        break;
    case ACCEL_BNDW_31:
        retval = bma2x2_set_bw(BMA2x2_BW_31_25HZ);
        break;
    case ACCEL_BNDW_63:
        retval = bma2x2_set_bw(BMA2x2_BW_62_50HZ);
        break;
    case ACCEL_BNDW_125:
        retval = bma2x2_set_bw(BMA2x2_BW_125HZ);
        break;
    case ACCEL_BNDW_250:
        retval = bma2x2_set_bw(BMA2x2_BW_250HZ);
        break;
    case ACCEL_BNDW_500:
        retval = bma2x2_set_bw(BMA2x2_BW_500HZ);
        break;
    case ACCEL_BNDW_1000:
        retval = bma2x2_set_bw(BMA2x2_BW_1000HZ);
        break;
    default:
        retval = bma2x2_set_bw(BMA2x2_BW_7_81HZ);
        break;
    }

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);

    // Set accelerometer range.
    switch(m_p.accel_range) {
    case ACCEL_RANGE_2G:
        retval = bma2x2_set_range(BMA2x2_RANGE_2G);
        break;
    case ACCEL_RANGE_4G:
        retval = bma2x2_set_range(BMA2x2_RANGE_4G);
        break;
    case ACCEL_RANGE_8G:
        retval = bma2x2_set_range(BMA2x2_RANGE_8G);
        break;
    case ACCEL_RANGE_16G:
        retval = bma2x2_set_range(BMA2x2_RANGE_16G);
        break;
    default:
        retval = bma2x2_set_range(BMA2x2_RANGE_2G);
        break;
    }

    if(retval != 0)
        APP_ERROR_CHECK_BOOL(0);
}

/// Reloads configurations from settings.

void accelerometer_reload_configs(uint8_t thres_high_G,uint8_t thres_slope)
{
    set_accelerometer_main_parameters();
    set_slope_interruption_parameters(thres_slope);
    set_high_g_interruption_parameters(thres_high_G);
}

/**
 * @brief accelerometer_init TWI and BMA250E initialization.
 * @param slope_interruption_signal Will be executed in interrupt context.
 * @param HIGH_G_interruption_signal Will be executed in interrupt context.
 * @return NRF_ERROR_INVALID_PARAM if any of the supplied parameters is NULL.
 * @return NRF_ERROR_INTERNAL if we failed to communicate with the accelerometer.
 */

uint32_t accelerometer_init(struct bma4_dev* bma4)

{
    // Communication status.
    int32_t retval = ERROR;

    // The I²C module should be already inited.
    retval = i2c_is_inited();
    if(retval != NRF_SUCCESS)
        return retval;

//    if(motion_detected_signal == NULL)
//        return NRF_ERROR_INVALID_PARAM;

//    if(high_G_signal == NULL)
//        return NRF_ERROR_INVALID_PARAM;


//    motion_detected_task = motion_detected_signal;
//    high_G_task = high_G_signal;

//    retval = app_timer_create(&slope_polling_timer, APP_TIMER_MODE_SINGLE_SHOT,
//                              check_if_we_are_moving);
//    if(retval != NRF_SUCCESS)
//        return retval;

//    detecting_movement = 0;
//    slope_int_arrived = 0;

//    // BMA2X2 instance struct initialization.
//    bma2x2.dev_addr = BMA2x2_I2C_ADDR1;
//    bma2x2.bus_write = bma_write;
//    bma2x2.bus_read = bma_read;
//    bma2x2.burst_read = bma_burst_read;
//    bma2x2.delay_msec = nrf_delay_ms;

//    // BMA2X2 initialization.
//    retval = bma2x2_init(&bma2x2);

//    retval += bma2x2_soft_rst();  // return configuration setting to default values

    //BMA4 instance struct initialization.


    struct bma4_axes_remap axes_remap = {
        .x_axis = BMA4_MAP_X_AXIS, .x_axis_sign = BMA4_MAP_POSITIVE, .y_axis = BMA4_MAP_Y_AXIS,
        .y_axis_sign = BMA4_MAP_POSITIVE, .z_axis = BMA4_MAP_Z_AXIS, .z_axis_sign = BMA4_MAP_POSITIVE
    };

    bma4->chip_id = BMA4_CHIP_ID_ADDR;
    bma4->bus_read = &bma_read;
    bma4->bus_write = &bma_write;
    bma4->intf = BMA4_I2C_INTF;
    bma4->delay_us = &accelerometer_delay_us;
    bma4->intf_ptr = BMA4_I2C_ADDR_PRIMARY;
    bma4->remap = axes_remap;
    bma4->resolution = BMA4_16_BIT_RESOLUTION;


    retval = bma4_init(bma4);
    retval += bma4_soft_reset(bma4);

    // Wait for BMA to start up
    nrf_delay_ms(5);

//    // Set the power mode as NORMAL
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

//    // Watchdog disabled.
//    retval += bma2x2_set_i2c_wdt(BMA2x2_ACCEL_I2C_SELECT, 0x00);

//    /*
//     * Set data source for different events, filtered (according to defined
//     * bandwidth) or non filtered (2Ksps). Register 0x1E.
//     */
//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_DATA, NON_FILTERED_SRC);

//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION, NON_FILTERED_SRC);
//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_TAP, NON_FILTERED_SRC);
//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_SLOPE, FILTERED_SRC);
//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_HIGH_G, FILTERED_SRC);
//    retval += bma2x2_set_source(BMA2x2_ACCEL_SOURCE_LOW_G, NON_FILTERED_SRC);


//    /**************************************************************************
//     * Interrupts, BMA side.
//     **************************************************************************/

//    // Set interrupt outputs, active high level, 1msec pulse, push pull.
//    retval += bma2x2_set_intr_output_type(BMA2x2_ACCEL_INTR1_OUTPUT, PUSH_PULL);

//    // INT1/INT2 outputs push pull.
//    retval += bma2x2_set_intr_output_type(BMA2x2_ACCEL_INTR2_OUTPUT, PUSH_PULL);

//    /*
//     * 50 ms interrupt outputs. The nRF51822 does not has a latch register for
//     * interruptions. Yeah, really, see
//     * <https://devzone.nordicsemi.com/question/59432/handling-multiple-gpio-pin-interrupts/?answer=59460#post-id-59460>
//     */
//    retval += bma2x2_set_latch_intr(BMA2x2_LATCH_DURN_50MS);

//    // INT1/INT2 outputs active high.
//    retval += bma2x2_set_intr_level(BMA2x2_ACCEL_INTR1_LEVEL,ACTIVE_HIGH);
//    retval += bma2x2_set_intr_level(BMA2x2_ACCEL_INTR2_LEVEL,ACTIVE_HIGH);

//    /*
//     * Set interrupt mapping to INT1/INT2 output pins.
//     * High G → INT1.
//     * Slope → INT2.
//     */
//    retval += bma2x2_set_intr_high_g(BMA2x2_INTR1_HIGH_G,INTR_ENABLE);
//    retval += bma2x2_set_intr_slope(BMA2x2_ACCEL_INTR2_SLOPE, INTR_ENABLE);


//    // Disable non used interruptions.

//    retval += bma2x2_set_intr_enable(DOUBLE_TAP_INTR, INTR_DISABLE);
//    retval += bma2x2_set_intr_enable(SINGLE_TAP_INTR, INTR_DISABLE);
//    retval += bma2x2_set_intr_enable(BMA2x2_LOW_G_INTR, INTR_DISABLE);
//    retval += bma2x2_set_slow_no_motion(BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SEL, INTR_DISABLE);



//    if(retval != 0)
//        return NRF_ERROR_INTERNAL;

//    // Load configurations.
//    accelerometer_reload_configs(settings_get_thres_am_high_G(),settings_get_thres_am_slope_G());


//    /**************************************************************************
//     * Interrupts, nRF side.
//     **************************************************************************/

//    // Set up the interrupt GPIOs as inputs.
//    nrf_gpio_cfg_input(BMA_INT1_PIN,NRF_GPIO_PIN_PULLDOWN);
//    nrf_gpio_cfg_input(BMA_INT2_PIN,NRF_GPIO_PIN_PULLDOWN);

//    /*
//     * Mask for selecting the interrupts to be taken into account in the
//     * GPIOTE app.
//     */
//    uint32_t low_to_high_mask = (1 << BMA_INT1_PIN) | (1 << BMA_INT2_PIN);

//    // Register us as a user.
//    app_gpiote_user_register(&accel_gpiote_id, low_to_high_mask, 0,
//                             interrupt_event_handler);

//    retval = accelerometer_enable_interrupts();

//    // Set the power mode as LOW_POWER_MODE 1 (SUSPEND Mode + NORMAL Mode).
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_LOWPOWER1);
//    retval += bma2x2_set_sleep_timer_mode(0); //enable EventDrivenSampling mode(EDT)
//    retval += bma2x2_set_sleep_durn(BMA2x2_SLEEP_DURN_1S);

//    if(retval != NRF_SUCCESS)
//        return retval;

    return NRF_SUCCESS;
}

/**
 * @brief accelerometer_set_signals
 * @param motion_detected_signal Will be executed in interrupt context.
 * @param no_motion_signal Will be executed in interrupt context.
 */
void accelerometer_set_signals(task motion_detected_signal, task high_G_signal)

{
//    if(motion_detected_signal == NULL)
//        APP_ERROR_CHECK_BOOL(0);

//    if(high_G_signal == NULL)
//        APP_ERROR_CHECK_BOOL(0);

//    motion_detected_task = motion_detected_signal;
//    high_G_task = high_G_signal;
}




/**
 * @brief accelerometer_get_alarm_movement.
 * @return alarm_movement : 0:no_motion 1:garbage collection 2:crash
 */

uint8_t accelerometer_get_alarm_movement(){

    return alarm_movement;
}

/**
 * @brief accelerometer_reset_alarm_movement.
 * Reset alarm: alarm_movement=0
 */

void  accelerometer_reset_alarm_movement(){

    alarm_movement=0;
}


int16_t accelerometer_get_acceleration_axis_z()
{
    int16_t acel_z=0;
//    int16_t retval=bma2x2_read_accel_z(&acel_z);


//    if(retval != SUCCESS)
//        return INT16_MAX;

    return acel_z;

}

int16_t accelerometer_get_acceleration_axis_y()
{
    int16_t acel_y=0;
//    int16_t retval=bma2x2_read_accel_y(&acel_y);

//    if(retval != SUCCESS)
//        return INT16_MAX;

    return acel_y;

}

int16_t accelerometer_get_acceleration_axis_x()
{
    int16_t acel_x=0;
//    int16_t retval=bma2x2_read_accel_x(&acel_x);

//    if(retval != SUCCESS)
//        return INT16_MAX;

    return acel_x;

}

uint32_t accelerometer_enable_interrupts()
{
    // Enable this user.
//    return app_gpiote_user_enable(accel_gpiote_id);
    return NRF_SUCCESS;
}

uint32_t accelerometer_disable_interrupts()
{
    // Disable the user.
//    return app_gpiote_user_disable(accel_gpiote_id);
       return NRF_SUCCESS;
}

uint32_t set_accelerometer_mode_normal(){
//    int32_t retval = ERROR;
//    retval = bma2x2_soft_rst();
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
//    if(retval != NRF_SUCCESS)
//        return retval;

    return NRF_SUCCESS;
}

uint32_t set_accelerometer_mode_low_power1(){
//    int32_t retval = ERROR;
//    retval = bma2x2_soft_rst();
//  //  retval += bma2x2_set_power_mode(BMA2x2_MODE_SUSPEND);
//    // Set the power mode as LOW_POWER_MODE 1 (SUSPEND Mode + NORMAL Mode).
////    retval += bma2x2_set_power_mode(BMA2x2_MODE_LOWPOWER1);
////    retval += bma2x2_set_sleep_timer_mode(0); //enable EventDrivenSampling(EDT)
////    retval += bma2x2_set_sleep_durn(BMA2x2_SLEEP_DURN_50MS);
//    if(retval != NRF_SUCCESS)
//        return retval;

    return NRF_SUCCESS;
}

uint32_t  set_accelerometer_mode_travel(){

//    // Communication status.
//    int32_t retval = ERROR;

//    // The I²C module should be already inited.
//    retval = i2c_is_inited();
//    if(retval != NRF_SUCCESS)
//        return retval;

//    // BMA2X2 instance struct initialization.
//    bma2x2.dev_addr = BMA2x2_I2C_ADDR1;
//    bma2x2.bus_write = bma_write;
//    bma2x2.bus_read = bma_read;
//    bma2x2.burst_read = bma_burst_read;
//    bma2x2.delay_msec = nrf_delay_ms;

//    // BMA2X2 initialization.
//    retval = bma2x2_init(&bma2x2);
//    retval += bma2x2_soft_rst();

//    // Wait for BMA to start up
//    nrf_delay_ms(5);

//    // Watchdog disabled.
//    retval += bma2x2_set_i2c_wdt(BMA2x2_ACCEL_I2C_SELECT, 0x00);

//    // Set the power mode as DEEP_SLEEP the lowest possible power consumption.
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);

//    if(retval != NRF_SUCCESS)
//        return retval;

    return NRF_SUCCESS;
}

int16_t accelerometer_get_temperature()
{
//    static int8_t temp;
//    int8_t retval=-1;
//    static int8_t retry;

//     while((retval!=SUCCESS) && (retry<5)){
//         retval = bma2x2_read_temp(&temp);
//         retry++;
//     }

//     if(retval != SUCCESS)
//         return INT16_MAX;

//    int16_t temperatura = (5 * (int16_t)temp) + 230;

//    return temperatura;

    return NRF_SUCCESS;
}

/**
 * @brief accelerometer_get_current_temperature
 * @return The accelerometer's temperature in 10*ºC.
 */
int16_t accelerometer_get_current_temperature()
{
//    struct bma2x2_accel_data_temp bma_xyzt_data;
//    uint32_t retval = get_accel(&bma_xyzt_data);

//    if(retval != SUCCESS)
//        return INT16_MAX;

//    /*
//     * Transform the temperature in 10*ºC.
//     * Temperature is in 0.5 ºC, two's complement, with value 0x0 == 23 ºC.
//     */
//    int16_t temp = (5 * (int16_t)bma_xyzt_data.temp) + 230;
//   /*
//    * bma_xyzt_data return -1 if occured a error
//    *
//    * */
//    return temp;
      return NRF_SUCCESS;
}

uint8_t accelerometer_alarm_temperature(const int16_t current_temperature,const int16_t last_temperature,const uint16_t threshold){
//    static uint16_t delta_temperature;
//    static uint8_t alarm_temperature;

//    if(current_temperature==INT16_MAX)
//        return alarm_temperature;

//    //take positive delta , because negative delta is not important.
//    if(current_temperature>=last_temperature){
//        delta_temperature = current_temperature-last_temperature;
//        if(delta_temperature>= threshold){
//            alarm_temperature=1;
//            return alarm_temperature;
//         }
//    }

//    if(current_temperature<0){
//        alarm_temperature=2;

//        return alarm_temperature;
//    }

//   return alarm_temperature;

   return NRF_SUCCESS;
    /*
    *We return 0 if temperature is OK.
    * 1 if it is exceeded with the positive threshold.
    * 2 if the temperature drops below zero degrees Celsius
    */
}

uint32_t set_accelerometer_threshold_movement(){

//    // Communication status.
//    int32_t retval = ERROR;

//    // The I²C module should be already inited.
//    retval = i2c_is_inited();
//    if(retval != NRF_SUCCESS)
//        return retval;

//    // BMA2X2 instance struct initialization.
//    bma2x2.dev_addr = BMA2x2_I2C_ADDR1;
//    bma2x2.bus_write = bma_write;
//    bma2x2.bus_read = bma_read;
//    bma2x2.burst_read = bma_burst_read;
//    bma2x2.delay_msec = nrf_delay_ms;

//    // BMA2X2 initialization.
//    retval = bma2x2_init(&bma2x2);

//    // Wait for BMA to start up
//    nrf_delay_ms(5);

//    // Set the power mode as NORMAL
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

//    // Load configurations.
//    accelerometer_reload_configs(settings_get_thres_am_high_G(),settings_get_thres_am_slope_G());

//    // Set the power mode as LOW_POWER_MODE 1 (SUSPEND Mode + NORMAL Mode).
//    retval += bma2x2_set_power_mode(BMA2x2_MODE_LOWPOWER1);
//    retval += bma2x2_set_sleep_timer_mode(0); //enable EventDrivenSampling mode(EDT)
//    retval += bma2x2_set_sleep_durn(BMA2x2_SLEEP_DURN_1S);

//    if(retval != NRF_SUCCESS)
//        return retval;

    return NRF_SUCCESS;
}

/**
 * @}
 */

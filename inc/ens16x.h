/*
 * ens16x.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS16X_H__
#define __ENS16X_H__

#ifndef ENS16X_DRIVER_DISABLE_FLAGS_FILE
#include "ens16x_driver_flags.h"
#endif
#include "error.h"
#include "types.h"

/*** ENS16X structures ***/

/*!******************************************************************
 * \enum ENS16X_status_t
 * \brief ENS16X driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    ENS16X_SUCCESS = 0,
    ENS16X_ERROR_NULL_PARAMETER,
    ENS16X_ERROR_OPERATING_MODE,
    ENS16X_ERROR_INPUT_TEMPERATURE,
    ENS16X_ERROR_INPUT_HUMIDITY,
    // Low level drivers errors.
    ENS16X_ERROR_BASE_I2C = ERROR_BASE_STEP,
    ENS16X_ERROR_BASE_DELAY = (ENS16X_ERROR_BASE_I2C + ENS16X_DRIVER_I2C_ERROR_BASE_LAST),
    // Last base value.
    ENS16X_ERROR_BASE_LAST = (ENS16X_ERROR_BASE_DELAY + ENS16X_DRIVER_DELAY_ERROR_BASE_LAST)
} ENS16X_status_t;

#ifndef ENS16X_DRIVER_DISABLE

/*******************************************************************/
typedef enum {
    ENS16X_OPERATING_MODE_DEEP_SLEEP = 0x00,
    ENS16X_OPERATING_MODE_IDLE = 0x01,
    ENS16X_OPERATING_MODE_STANDARD = 0x02,
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    ENS16X_OPERATING_MODE_LOW_POWER = 0x03,
    ENS16X_OPERATING_MODE_ULTRA_LOW_POWER = 0x04,
#endif
    ENS16X_OPERATING_MODE_RESET = 0xF0
} ENS16X_operating_mode_t;

/*!******************************************************************
 * \enum ENS16X_validity_flag_t
 * \brief ENS16X validity flag values mapping.
 *******************************************************************/
typedef enum {
    ENS16X_VALIDITY_FLAG_NORMAL_OPERATION = 0b00,
    ENS16X_VALIDITY_FLAG_WARM_UP_PHASE = 0b01,
    ENS16X_VALIDITY_FLAG_INITIAL_STARTUP_PHASE = 0b10,
    ENS16X_VALIDITY_FLAG_INVALID_OUTPUT = 0b11
} ENS16X_validity_flag_t;

/*!******************************************************************
 * \enum ENS16X_device_status_t
 * \brief ENS16X chip status byte.
 *******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned statas :1;
        unsigned stater :1;
        unsigned reserved :2;
        unsigned validity_flag :2;
        unsigned newdat :1;
        unsigned newgpr :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} ENS16X_device_status_t;

/*!******************************************************************
 * \enum ENS16X_air_quality_data_t
 * \brief ENS16X output data structure.
 *******************************************************************/
typedef struct {
    int32_t aqi_uba;
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    int32_t aqi_s;
#endif
    int32_t tvoc_ppb;
    int32_t eco2_ppm;
} ENS16X_air_quality_data_t;

/*** ENS16X functions ***/

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_init(void)
 * \brief Init ENS16X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_init(void);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_de_init(void)
 * \brief Release ENS16X driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_de_init(void);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_get_device_status(uint8_t i2c_address, ENS16X_device_status_t* device_status)
 * \brief Get ENS16X chip status.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  device_status: Pointer to byte that will contain the current chip status.
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_get_device_status(uint8_t i2c_address, ENS16X_device_status_t* device_status);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_set_operating_mode(uint8_t i2c_address, ENS16X_operating_mode_t operating_mode)
 * \brief Set sensor state.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   operating_mode: operating mode to select.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_set_operating_mode(uint8_t i2c_address, ENS16X_operating_mode_t operating_mode);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_set_temperature_humidity(uint8_t i2c_address, int32_t temperature_tenth_degrees, int32_t humidity_percent)
 * \brief Write temperature and humidity compensation.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   temperature_tenth_degrees: Ambient temperature in 1/10 Celsius degrees.
 * \param[in]   humidity_percent: Ambient relative humidity in percent.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_set_temperature_humidity(uint8_t i2c_address, int32_t temperature_tenth_degrees, int32_t humidity_percent);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_read_air_quality(uint8_t i2c_address, ENS16X_air_quality_data_t* air_quality_data)
 * \brief Read air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  air_quality_data: Pointer to the air quality data structure.
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_read_air_quality(uint8_t i2c_address, ENS16X_air_quality_data_t* air_quality_data);

/*******************************************************************/
#define ENS16X_exit_error(base) { ERROR_check_exit(ens16x_status, ENS16X_SUCCESS, base) }

/*******************************************************************/
#define ENS16X_stack_error(base) { ERROR_check_stack(ens16x_status, ENS16X_SUCCESS, base) }

/*******************************************************************/
#define ENS16X_stack_exit_error(base, code) { ERROR_check_stack_exit(ens16x_status, ENS16X_SUCCESS, base, code) }

#endif /* ENS16X_DRIVER_DISABLE */

#endif /* __ENS16X_H__ */

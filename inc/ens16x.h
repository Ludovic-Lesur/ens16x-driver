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
    ENS16X_ERROR_SENSING_MODE,
    ENS16X_ERROR_INPUT_TEMPERATURE,
    ENS16X_ERROR_INPUT_HUMIDITY,
    // Low level drivers errors.
    ENS16X_ERROR_BASE_I2C = ERROR_BASE_STEP,
    // Last base value.
    ENS16X_ERROR_BASE_LAST = (ENS16X_ERROR_BASE_I2C + ENS16X_DRIVER_I2C_ERROR_BASE_LAST)
} ENS16X_status_t;

#ifndef ENS16X_DRIVER_DISABLE

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
 * \enum ENS16X_device_status_t
 * \brief ENS16X chip status byte.
 *******************************************************************/
typedef enum {
    ENS16X_SENSING_MODE_STANDARD = 0,
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    ENS16X_SENSING_MODE_LOW_POWER,
    ENS16X_SENSING_MODE_ULTRA_LOW_POWER,
#endif
    ENS16X_SENSING_MODE_LAST
} ENS16X_sensing_mode_t;

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
 * \fn ENS16X_status_t ENS16X_start_acquisition(uint8_t i2c_address, ENS16X_sensing_mode_t sensing_mode, int32_t temperature_tenth_degrees, int32_t humidity_percent)
 * \brief Start air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   sensing_mode: Gas sensing mode to select.
 * \param[in]   temperature_tenth_degrees: Ambient temperature in 1/10 Celsius degrees.
 * \param[in]   humidity_percent: Ambient relative humidity in percent.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_start_acquisition(uint8_t i2c_address, ENS16X_sensing_mode_t sensing_mode, int32_t temperature_tenth_degrees, int32_t humidity_percent);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_stop_acquisition(uint8_t i2c_address)
 * \brief Stop air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_stop_acquisition(uint8_t i2c_address);

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

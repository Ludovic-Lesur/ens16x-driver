/*
 * ens160.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS160_H__
#define __ENS160_H__

#ifndef ENS160_DRIVER_DISABLE_FLAGS_FILE
#include "ens160_driver_flags.h"
#endif
#include "error.h"
#include "types.h"

/*** ENS160 structures ***/

/*!******************************************************************
 * \enum ENS160_status_t
 * \brief ENS160 driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    ENS160_SUCCESS = 0,
    ENS160_ERROR_NULL_PARAMETER,
    ENS160_ERROR_INPUT_TEMPERATURE,
    ENS160_ERROR_INPUT_HUMIDITY,
    // Low level drivers errors.
    ENS160_ERROR_BASE_I2C = ERROR_BASE_STEP,
    // Last base value.
    ENS160_ERROR_BASE_LAST = (ENS160_ERROR_BASE_I2C + ENS160_DRIVER_I2C_ERROR_BASE_LAST)
} ENS160_status_t;

#ifndef ENS160_DRIVER_DISABLE

/*!******************************************************************
 * \enum ENS160_device_status_t
 * \brief ENS160 chip status byte.
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
} ENS160_device_status_t;

/*** ENS160 functions ***/

/*!******************************************************************
 * \fn ENS160_status_t ENS160_init(void)
 * \brief Init ENS160 driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_init(void);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_de_init(void)
 * \brief Release ENS160 driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_de_init(void);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_get_device_status(uint8_t i2c_address, ENS160_device_status_t* device_status)
 * \brief Get ENS160 chip status.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  device_status: Pointer to byte that will contain the current chip status.
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_get_device_status(uint8_t i2c_address, ENS160_device_status_t* device_status);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_start_acquisition(uint8_t i2c_address, int32_t temperature_degrees, int32_t humidity_percent)
 * \brief Start air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[in]   temperature_degrees: Ambient temperature in degrees.
 * \param[in]   humidity_percent: Ambient relative humidity in percent.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_start_acquisition(uint8_t i2c_address, int32_t temperature_degrees, int32_t humidity_percent);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_stop_acquisition(uint8_t i2c_address)
 * \brief Stop air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_stop_acquisition(uint8_t i2c_address);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_read_air_quality(uint8_t i2c_address, int32_t* aqi_uba, int32_t* tvoc_ppb, int32_t* eco2_ppm)
 * \brief Read air quality measurements.
 * \param[in]   i2c_address: I2C address of the sensor.
 * \param[out]  aqi_uba: Pointer to the air quality index.
 * \param[out]  tvoc_ppb: Pointer to the TVOC concentration.
 * \param[out]  eco2_ppm: Pointer to the equivalent CO2 concentration.
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_read_air_quality(uint8_t i2c_address, int32_t* aqi_uba, int32_t* tvoc_ppb, int32_t* eco2_ppm);

/*******************************************************************/
#define ENS160_exit_error(base) { ERROR_check_exit(ens160_status, ENS160_SUCCESS, base) }

/*******************************************************************/
#define ENS160_stack_error(base) { ERROR_check_stack(ens160_status, ENS160_SUCCESS, base) }

/*******************************************************************/
#define ENS160_stack_exit_error(base, code) { ERROR_check_stack_exit(ens160_status, ENS160_SUCCESS, base, code) }

#endif /* ENS160_DRIVER_DISABLE */

#endif /* __ENS160_H__ */

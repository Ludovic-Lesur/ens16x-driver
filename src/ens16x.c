/*
 * ens16x.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens16x.h"

#ifndef ENS16X_DRIVER_DISABLE_FLAGS_FILE
#include "ens16x_driver_flags.h"
#endif
#include "ens16x_hw.h"
#include "types.h"

#ifndef ENS16X_DRIVER_DISABLE

/*** ENS16X local macros ***/

#define ENS16X_IDLE_DELAY_MS                10
#define ENS16X_COMPENSATION_DELAY_MS        10

#define ENS16X_WRITE_COMMAND_BUFFER_SIZE    2

#define ENS16X_TEMP_IN_MIN                  (-400)
#define ENS16X_TEMP_IN_MAX                  850
#define ENS16X_TEMP_IN_FACTOR               10
#define ENS16X_TEMP_IN_SLOPE                64
#define ENS16X_TEMP_IN_OFFSET               174816

#define ENS16X_RH_IN_MIN                    0
#define ENS16X_RH_IN_MAX                    100
#define ENS16X_RH_IN_SLOPE                  512

/*** ENS16X local structures ***/

/*******************************************************************/
typedef enum {
    ENS16X_REGISTER_PART_ID = 0x00,
    ENS16X_REGISTER_OPMODE = 0x10,
    ENS16X_REGISTER_CONFIG = 0x11,
    ENS16X_REGISTER_COMMAND = 0x12,
    ENS16X_REGISTER_TEMP_IN = 0x13,
    ENS16X_REGISTER_RH_IN = 0x15,
    ENS16X_REGISTER_DEVICE_STATUS = 0x20,
    ENS16X_REGISTER_DATA_AQI_UBA = 0x21,
    ENS16X_REGISTER_DATA_TVOC = 0x22,
    ENS16X_REGISTER_DATA_ECO2 = 0x24,
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    ENS16X_REGISTER_DATA_AQI_S = 0x26,
#endif
    ENS16X_REGISTER_DATA_T = 0x30,
    ENS16X_REGISTER_DATA_RH = 0x32,
    ENS16X_REGISTER_DATA_MISR = 0x38,
    ENS16X_REGISTER_GPR_WRITE = 0x40,
    ENS16X_REGISTER_GPR_READ = 0x48,
    ENS16X_REGISTER_LAST
} ENS16X_register_t;

/*******************************************************************/
typedef enum {
    ENS16X_OPERATING_MODE_DEEP_SLEEP = 0x00,
    ENS16X_OPERATING_MODE_IDLE = 0x01,
    ENS16X_OPERATING_MODE_STANDARD = 0x02,
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    ENS16X_OPERATING_MODE_LOW_POWER = 0x03,
    ENS16X_OPERATING_MODE_ULTRA_LOW_POWER = 0x04,
#endif
    ENS16X_OPERATING_MODE_RESET = 0xF0,
    ENS16X_OPERATING_MODE_LAST
} ENS16X_operating_mode_t;

/*** ENS16X local functions ***/

/*******************************************************************/
static ENS16X_status_t _ENS16X_write_register(uint8_t i2c_address, ENS16X_register_t register_address, uint8_t value) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    uint8_t register_write_command[ENS16X_WRITE_COMMAND_BUFFER_SIZE] = { register_address, value };
    // I2C transfer.
    status = ENS16X_HW_i2c_write(i2c_address, register_write_command, ENS16X_WRITE_COMMAND_BUFFER_SIZE, 1);
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static ENS16X_status_t _ENS16X_read_register(uint8_t i2c_address, ENS16X_register_t register_address, uint8_t* value) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    ENS16X_register_t local_addr = register_address;
    // I2C transfer.
    status = ENS16X_HW_i2c_write(i2c_address, &local_addr, 1, 1);
    if (status != ENS16X_SUCCESS) goto errors;
    status = ENS16X_HW_i2c_read(i2c_address, value, 1);
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*** ENS16X functions ***/

/*******************************************************************/
ENS16X_status_t ENS16X_init(void) {
    // Local variables.humidity_percent
    ENS16X_status_t status = ENS16X_SUCCESS;
    // Init hardware interface.
    status = ENS16X_HW_init();
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS16X_status_t ENS16X_de_init(void) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    // Release hardware interface.
    status = ENS16X_HW_de_init();
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS16X_status_t ENS16X_get_device_status(uint8_t i2c_address, ENS16X_device_status_t* device_status) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    uint8_t reg_value = 0;
    // Check parameter.
    if (device_status == NULL) {
        status = ENS16X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read register.
    status = _ENS16X_read_register(i2c_address, ENS16X_REGISTER_DEVICE_STATUS, &reg_value);
    if (status != ENS16X_SUCCESS) goto errors;
    // Update output.
    (device_status->all) = reg_value;
errors:
    return status;
}

/*******************************************************************/
ENS16X_status_t ENS16X_start_acquisition(uint8_t i2c_address, ENS16X_sensing_mode_t sensing_mode, int32_t temperature_tenth_degrees, int32_t humidity_percent) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    ENS16X_operating_mode_t op_mode = ENS16X_OPERATING_MODE_DEEP_SLEEP;
    int32_t temp_in = 0;
    int32_t rh_in = 0;
    // Check parameters.
    if ((temperature_tenth_degrees < ENS16X_TEMP_IN_MIN) || (temperature_tenth_degrees > ENS16X_TEMP_IN_MAX)) {
        status = ENS16X_ERROR_INPUT_TEMPERATURE;
        goto errors;
    }
    if ((humidity_percent < ENS16X_RH_IN_MIN) | (humidity_percent > ENS16X_RH_IN_MAX)) {
        status = ENS16X_ERROR_INPUT_HUMIDITY;
        goto errors;
    }
    // Compute registers value.
    temp_in = (((temperature_tenth_degrees * ENS16X_TEMP_IN_SLOPE) + ENS16X_TEMP_IN_OFFSET) / (ENS16X_TEMP_IN_FACTOR));
    rh_in = (humidity_percent * ENS16X_RH_IN_SLOPE);
    // Select operating mode.
    switch (sensing_mode) {
    case ENS16X_SENSING_MODE_STANDARD:
        op_mode = ENS16X_OPERATING_MODE_STANDARD;
        break;
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    case ENS16X_SENSING_MODE_LOW_POWER:
        op_mode = ENS16X_OPERATING_MODE_LOW_POWER;
        break;
    case ENS16X_SENSING_MODE_ULTRA_LOW_POWER:
        op_mode = ENS16X_OPERATING_MODE_ULTRA_LOW_POWER;
        break;
#endif
    default:
        status = ENS16X_ERROR_SENSING_MODE;
        goto errors;
    }
    // Wake-up sensor.
    status = _ENS16X_write_register(i2c_address, ENS16X_REGISTER_OPMODE, ENS16X_OPERATING_MODE_IDLE);
    if (status != ENS16X_SUCCESS) goto errors;
    status = ENS16X_HW_delay_milliseconds(ENS16X_IDLE_DELAY_MS);
    if (status != ENS16X_SUCCESS) goto errors;
    // Write compensation registers.
    status = _ENS16X_write_register(i2c_address, (ENS16X_REGISTER_TEMP_IN + 0), (uint8_t) (temp_in >> 0));
    if (status != ENS16X_SUCCESS) goto errors;
    status = _ENS16X_write_register(i2c_address, (ENS16X_REGISTER_TEMP_IN + 1), (uint8_t) (temp_in >> 8));
    if (status != ENS16X_SUCCESS) goto errors;
    status = _ENS16X_write_register(i2c_address, (ENS16X_REGISTER_RH_IN + 0), (uint8_t) (rh_in >> 0));
    if (status != ENS16X_SUCCESS) goto errors;
    status = _ENS16X_write_register(i2c_address, (ENS16X_REGISTER_RH_IN + 1), (uint8_t) (rh_in >> 8));
    if (status != ENS16X_SUCCESS) goto errors;
    status = ENS16X_HW_delay_milliseconds(ENS16X_COMPENSATION_DELAY_MS);
    if (status != ENS16X_SUCCESS) goto errors;
    // Start acquisition.
    status = _ENS16X_write_register(i2c_address, ENS16X_REGISTER_OPMODE, op_mode);
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS16X_status_t ENS16X_stop_acquisition(uint8_t i2c_address) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    // Put sensor in deep-sleep mode.
    status = _ENS16X_write_register(i2c_address, ENS16X_REGISTER_OPMODE, ENS16X_OPERATING_MODE_DEEP_SLEEP);
    if (status != ENS16X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS16X_status_t ENS16X_read_air_quality(uint8_t i2c_address, ENS16X_air_quality_data_t* air_quality_data) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    uint8_t generic_u8 = 0;
    // Check parameters.
    if (air_quality_data == NULL) {
        status = ENS16X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read AQI UBA.
    status = _ENS16X_read_register(i2c_address, ENS16X_REGISTER_DATA_AQI_UBA, &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->aqi_uba) = (generic_u8 & 0x07);
#ifdef ENS16X_DRIVER_DEVICE_ENS161
    // Read AQI S.
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_AQI_S + 0), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->aqi_s) = (int32_t) (generic_u8 << 0);
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_AQI_S + 1), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->aqi_s) |= (int32_t) (generic_u8 << 8);
#endif
    // Read TVOC.
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_TVOC + 0), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->tvoc_ppb) = (int32_t) (generic_u8 << 0);
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_TVOC + 1), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->tvoc_ppb) |= (int32_t) (generic_u8 << 8);
    // Read ECO2.
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_ECO2 + 0), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->eco2_ppm) = (int32_t) (generic_u8 << 0);
    status = _ENS16X_read_register(i2c_address, (ENS16X_REGISTER_DATA_ECO2 + 1), &generic_u8);
    if (status != ENS16X_SUCCESS) goto errors;
    (air_quality_data->eco2_ppm) |= (int32_t) (generic_u8 << 8);
errors:
    return status;
}

#endif /* ENS16X_DRIVER_DISABLE */

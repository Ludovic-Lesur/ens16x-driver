/*
 * ens160.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens160.h"

#ifndef ENS160_DRIVER_DISABLE_FLAGS_FILE
#include "ens160_driver_flags.h"
#endif
#include "ens160_hw.h"
#include "types.h"

#ifndef ENS160_DRIVER_DISABLE

/*** ENS160 local macros ***/

#define ENS160_WRITE_COMMAND_BUFFER_SIZE    2

#define ENS160_TEMP_IN_MIN                  (-40)
#define ENS160_TEMP_IN_MAX                  85
#define ENS160_TEMP_IN_FACTOR               64
#define ENS160_TEMP_IN_KELVIN_OFFSET        17482

#define ENS160_RH_IN_MIN                    0
#define ENS160_RH_IN_MAX                    100
#define ENS160_RH_IN_FACTOR                 512

/*** ENS160 local structures ***/

/*******************************************************************/
typedef enum {
    ENS160_REGISTER_PART_ID = 0x00,
    ENS160_REGISTER_OPMODE = 0x10,
    ENS160_REGISTER_CONFIG = 0x11,
    ENS160_REGISTER_COMMAND = 0x12,
    ENS160_REGISTER_TEMP_IN = 0x13,
    ENS160_REGISTER_RH_IN = 0x15,
    ENS160_REGISTER_DEVICE_STATUS = 0x20,
    ENS160_REGISTER_DATA_AQI = 0x21,
    ENS160_REGISTER_DATA_TVOC = 0x22,
    ENS160_REGISTER_DATA_ECO2 = 0x24,
    ENS160_REGISTER_DATA_T = 0x30,
    ENS160_REGISTER_DATA_RH = 0x32,
    ENS160_REGISTER_DATA_MISR = 0x38,
    ENS160_REGISTER_GPR_WRITE = 0x40,
    ENS160_REGISTER_GPR_READ = 0x48,
    ENS160_REGISTER_LAST
} ENS160_register_t;

/*******************************************************************/
typedef enum {
    ENS160_OPERATING_MODE_DEEP_SLEEP = 0x00,
    ENS160_OPERATING_MODE_IDLE = 0x01,
    ENS160_OPERATING_MODE_STANDARD = 0x02,
    ENS160_OPERATING_MODE_RESET = 0xF0,
    ENS160_OPERATING_MODE_LAST
} ENS160_operating_mode_t;

/*** ENS160 local functions ***/

/*******************************************************************/
static ENS160_status_t _ENS160_write_register(uint8_t i2c_address, ENS160_register_t register_address, uint8_t value) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    uint8_t register_write_command[ENS160_WRITE_COMMAND_BUFFER_SIZE] = { register_address, value };
    // I2C transfer.
    status = ENS160_HW_i2c_write(i2c_address, register_write_command, ENS160_WRITE_COMMAND_BUFFER_SIZE, 1);
    if (status != ENS160_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
static ENS160_status_t _ENS160_read_register(uint8_t i2c_address, ENS160_register_t register_address, uint8_t* value) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    ENS160_register_t local_addr = register_address;
    // I2C transfer.
    status = ENS160_HW_i2c_write(i2c_address, &local_addr, 1, 1);
    if (status != ENS160_SUCCESS) goto errors;
    status = ENS160_HW_i2c_read(i2c_address, value, 1);
    if (status != ENS160_SUCCESS) goto errors;
errors:
    return status;
}

/*** ENS160 functions ***/

/*******************************************************************/
ENS160_status_t ENS160_init(void) {
    // Local variables.humidity_percent
    ENS160_status_t status = ENS160_SUCCESS;
    // Init hardware interface.
    status = ENS160_HW_init();
    return status;
}

/*******************************************************************/
ENS160_status_t ENS160_de_init(void) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    // Release hardware interface.
    status = ENS160_HW_de_init();
    return status;
}

/*******************************************************************/
ENS160_status_t ENS160_start_acquisition(uint8_t i2c_address, int32_t temperature_degrees, int32_t humidity_percent) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    int32_t temp_in = 0;
    int32_t rh_in = 0;
    // Check parameters.
    if ((humidity_percent < ENS160_TEMP_IN_MIN) || (humidity_percent > ENS160_TEMP_IN_MAX)) {
        status = ENS160_ERROR_INPUT_TEMPERATURE;
        goto errors;
    }
    if ((humidity_percent < ENS160_RH_IN_MIN) | (humidity_percent > ENS160_RH_IN_MAX)) {
        status = ENS160_ERROR_INPUT_HUMIDITY;
        goto errors;
    }
    // Compute registers value.
    temp_in = (temperature_degrees * ENS160_TEMP_IN_FACTOR) + ENS160_TEMP_IN_KELVIN_OFFSET;
    rh_in = (humidity_percent * ENS160_RH_IN_FACTOR);
    // Wake-up sensor.
    status = _ENS160_write_register(i2c_address, ENS160_REGISTER_OPMODE, ENS160_OPERATING_MODE_IDLE);
    if (status != ENS160_SUCCESS) goto errors;
    // Write compensation registers.
    status = _ENS160_write_register(i2c_address, (ENS160_REGISTER_TEMP_IN + 0), (uint8_t) (temp_in >> 0));
    if (status != ENS160_SUCCESS) goto errors;
    status = _ENS160_write_register(i2c_address, (ENS160_REGISTER_TEMP_IN + 1), (uint8_t) (temp_in >> 8));
    if (status != ENS160_SUCCESS) goto errors;
    status = _ENS160_write_register(i2c_address, (ENS160_REGISTER_RH_IN + 0), (uint8_t) (rh_in >> 0));
    if (status != ENS160_SUCCESS) goto errors;
    status = _ENS160_write_register(i2c_address, (ENS160_REGISTER_RH_IN + 1), (uint8_t) (rh_in >> 8));
    if (status != ENS160_SUCCESS) goto errors;
    // Start acquisition.
    status = _ENS160_write_register(i2c_address, ENS160_REGISTER_OPMODE, ENS160_OPERATING_MODE_STANDARD);
    if (status != ENS160_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS160_status_t ENS160_stop_acquisition(uint8_t i2c_address) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    // Put sensor in deep-sleep mode.
    status = _ENS160_write_register(i2c_address, ENS160_REGISTER_OPMODE, ENS160_OPERATING_MODE_DEEP_SLEEP);
    if (status != ENS160_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ENS160_status_t ENS160_read_air_quality(uint8_t i2c_address, int32_t* aqi_uba, int32_t* tvoc_ppb, int32_t* eco2_ppm) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    uint8_t generic_u8 = 0;
    // Check parameters.
    if ((aqi_uba == NULL) || (tvoc_ppb == NULL) || (eco2_ppm == NULL)) {
        status = ENS160_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read AQI.
    status = _ENS160_read_register(i2c_address, ENS160_REGISTER_DATA_AQI, &generic_u8);
    if (status != ENS160_SUCCESS) goto errors;
    (*aqi_uba) = (generic_u8 & 0x07);
    // Read TVOC.
    status = _ENS160_read_register(i2c_address, (ENS160_REGISTER_DATA_TVOC + 0), &generic_u8);
    if (status != ENS160_SUCCESS) goto errors;
    (*tvoc_ppb) = (int32_t) (generic_u8 << 0);
    status = _ENS160_read_register(i2c_address, (ENS160_REGISTER_DATA_TVOC + 1), &generic_u8);
    if (status != ENS160_SUCCESS) goto errors;
    (*tvoc_ppb) |= (int32_t) (generic_u8 << 8);
    // Read ECO2.
    status = _ENS160_read_register(i2c_address, (ENS160_REGISTER_DATA_ECO2 + 0), &generic_u8);
    if (status != ENS160_SUCCESS) goto errors;
    (*eco2_ppm) = (int32_t) (generic_u8 << 0);
    status = _ENS160_read_register(i2c_address, (ENS160_REGISTER_DATA_ECO2 + 1), &generic_u8);
    if (status != ENS160_SUCCESS) goto errors;
    (*eco2_ppm) |= (int32_t) (generic_u8 << 8);
errors:
    return status;
}

#endif /* ENS160_DRIVER_DISABLE */

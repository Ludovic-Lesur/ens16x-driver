/*
 * ens16x_hw.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens16x_hw.h"

#ifndef ENS16X_DRIVER_DISABLE_FLAGS_FILE
#include "ens16x_driver_flags.h"
#endif
#include "ens16x.h"
#include "types.h"

#ifndef ENS16X_DRIVER_DISABLE

/*** ENS16X HW functions ***/

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_init(void) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_de_init(void) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    UNUSED(stop_flag);
    return status;
}

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    return status;
}

/*******************************************************************/
ENS16X_status_t __attribute__((weak)) ENS16X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    ENS16X_status_t status = ENS16X_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#endif /* ENS16X_DRIVER_DISABLE */

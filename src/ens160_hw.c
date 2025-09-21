/*
 * ens160_hw.c
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#include "ens160_hw.h"

#ifndef ENS160_DRIVER_DISABLE_FLAGS_FILE
#include "ens160_driver_flags.h"
#endif
#include "ens160.h"
#include "types.h"

#ifndef ENS160_DRIVER_DISABLE

/*** ENS160 HW functions ***/

/*******************************************************************/
ENS160_status_t __attribute__((weak)) ENS160_HW_init(void) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
ENS160_status_t __attribute__((weak)) ENS160_HW_de_init(void) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
ENS160_status_t __attribute__((weak)) ENS160_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    UNUSED(stop_flag);
    return status;
}

/*******************************************************************/
ENS160_status_t __attribute__((weak)) ENS160_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes) {
    // Local variables.
    ENS160_status_t status = ENS160_SUCCESS;
    /* To be implemented */
    UNUSED(i2c_address);
    UNUSED(data);
    UNUSED(data_size_bytes);
    return status;
}

#endif /* ENS160_DRIVER_DISABLE */

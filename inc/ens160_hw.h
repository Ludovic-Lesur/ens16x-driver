/*
 * ens160_hw.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS160_HW_H__
#define __ENS160_HW_H__

#ifndef ENS160_DRIVER_DISABLE_FLAGS_FILE
#include "ens160_driver_flags.h"
#endif
#include "ens160.h"
#include "types.h"

#ifndef ENS160_DRIVER_DISABLE

/*** ENS160 HW functions ***/

/*!******************************************************************
 * \fn ENS160_status_t ENS160_HW_init(void)
 * \brief Init ENS160 hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_HW_init(void);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_HW_de_init(void)
 * \brief Release ENS160 hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_HW_de_init(void);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data to sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[in]   stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn ENS160_status_t ENS160_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data from sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data_size_bytes: Number of bytes to read.
 * \param[out]  data: Byte array that will contain the read data.
 * \retval      Function execution status.
 *******************************************************************/
ENS160_status_t ENS160_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes);

#endif /* ENS160_DRIVER_DISABLE */

#endif /* __ENS160_HW_H__ */

/*
 * ens16x_hw.h
 *
 *  Created on: 21 sep. 2025
 *      Author: Ludo
 */

#ifndef __ENS16X_HW_H__
#define __ENS16X_HW_H__

#ifndef ENS16X_DRIVER_DISABLE_FLAGS_FILE
#include "ens16x_driver_flags.h"
#endif
#include "ens16x.h"
#include "types.h"

#ifndef ENS16X_DRIVER_DISABLE

/*** ENS16X HW functions ***/

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_HW_init(void)
 * \brief Init ENS16X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_HW_init(void);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_HW_de_init(void)
 * \brief Release ENS16X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_HW_de_init(void);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data to sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[in]   stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_HW_i2c_write(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data from sensor over I2C bus.
 * \param[in]   i2c_address: 7-bits sensor address.
 * \param[in]   data_size_bytes: Number of bytes to read.
 * \param[out]  data: Byte array that will contain the read data.
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_HW_i2c_read(uint8_t i2c_address, uint8_t* data, uint8_t data_size_bytes);

/*!******************************************************************
 * \fn ENS16X_status_t ENS16X_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]   delay_ms: Delay to wait in ms.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ENS16X_status_t ENS16X_HW_delay_milliseconds(uint32_t delay_ms);

#endif /* ENS16X_DRIVER_DISABLE */

#endif /* __ENS16X_HW_H__ */

/**
 * @file    TMC5160_SPI.h
 * @brief   TMC5160 SPI transport layer: low-level register read/write.
 */

#ifndef TMC5160_SPI_H
#define TMC5160_SPI_H

#include "TMC5160.h"

/**
 * @brief  Write a 32-bit value to a register as a 40-bit SPI datagram.
 * @param  htmc        Driver handle.
 * @param  reg_address Register address; bit 7 is set internally to select write mode.
 * @param  data        Value to write (sent MSB first).
 */
void TMC5160_SPI_Write(TMC5160_TypeDef *htmc, uint8_t reg_address, uint32_t data);

/**
 * @brief  Read a 32-bit value from a register.
 * @param  htmc        Driver handle.
 * @param  reg_address Register address; bit 7 is cleared internally to select read mode.
 * @return Register value, or 0 on invalid argument.
 * @note   Issues two datagrams: a TMC5160 read returns the register addressed
 *         by the previous frame.
 */
uint32_t TMC5160_SPI_Read(TMC5160_TypeDef *htmc, uint8_t reg_address);

#endif
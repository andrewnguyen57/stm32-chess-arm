/**
 * @file    TMC5160_SPI.c
 * @brief   TMC5160 SPI Communication
 *
 * Author: Andrew Nguyen
 * Date:   June 2026
 * 
 * References:
 * Trinamic TMC5160 Datasheet (see docs)
 *  Relevant page numbers are cited throughout the source.
 */

#include "TMC5160_SPI.h"

// --- SPI COMMUNICATION ---
// 40-bit datagram: 1 register byte + 4 data byte

void TMC5160_SPI_Write(TMC5160_TypeDef *htmc, uint8_t reg_address, uint32_t data)
{   
    if (htmc == NULL || htmc->hspi == NULL || htmc->cs.port == NULL) {return;}

    uint8_t tx_buff[5];

    tx_buff[0] = reg_address | 0x80 ; // Bit 7 = 1: WRITE MODE
    tx_buff[1] = (data >> 24) & 0xFF; 
    tx_buff[2] = (data >> 16) & 0xFF;
    tx_buff[3] = (data >> 8) & 0xFF;
    tx_buff[4] = (data >> 0) & 0xFF;

    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(htmc->hspi, tx_buff, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_SET);
}


uint32_t TMC5160_SPI_Read(TMC5160_TypeDef *htmc, uint8_t reg_address)
{
    if (htmc == NULL || htmc->hspi == NULL || htmc->cs.port == NULL) {return 0;}

    // Read returns previous requested datagram
    // Two transactions are needed to first queue a request register then read from next request
    uint8_t tx_buff[5] = {reg_address & 0x7F, 0, 0, 0, 0}; // Bit 7 = 0: READ MODE
    uint8_t rx_buff[5] = {0};

    // First transaction - TMC5160 gets the address but returns garbage
    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(htmc->hspi, tx_buff, rx_buff, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_SET);

    // Second transaction - TMC5160 returns actual data
    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(htmc->hspi, tx_buff, rx_buff, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(htmc->cs.port, htmc->cs.pin, GPIO_PIN_SET);

    // Reassemble the 32-bit payload
    uint32_t data = (uint32_t) rx_buff[1] << 24 | 
                    (uint32_t) rx_buff[2] << 16 | 
                    (uint32_t) rx_buff[3] << 8 | 
                    (uint32_t) rx_buff[4] << 0;
    
    return data;
}



/*
 * nRF24L01.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Morgan
 */

#include "nRF24L01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"

/*
 *
 */
void nRF24_QS(struct nRF24_Handle nRF24_H, uint8_t TX_mode)
{
	HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_RESET);	// Standby I mode
	HAL_Delay(20);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);
	nRF24_QSconfig(nRF24_H);

	if(!TX_mode)
	{
		nRF24_RegWrite(nRF24_H, 0x00, 0x0B); // PRX, Power ON, CRC, 1 byte
	}
	nRF24_FlushTX(nRF24_H);
	nRF24_RegWrite(nRF24_H, 0x07, 0x70);	// Clear TX/RX Interrupt

};

void nRF24_QSconfig(struct nRF24_Handle nRF24_H)
{
	nRF24_RegWrite(nRF24_H, 0x00, 0x0A); // PTX, Power ON, CRC, 1 byte
	nRF24_RegWrite(nRF24_H, 0x00, 0x0A); // PTX, Power ON, CRC, 1 byte
	nRF24_RegWrite(nRF24_H, 0x04, 0xFF); // 15 Retry, 4ms wait
	nRF24_RegWrite(nRF24_H, 0x06, 0x08); // -18dBm, 2Mbp
	for (uint8_t i = 0x11; i < 0x17 ; i++)	// All pipe size = 1 byte
	{
		nRF24_RegRead(nRF24_H, i);
	}
};






/*
 *  @brief Write value into the register address
 *
 * 	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *  @param address: the register address (please refer to nRF24L01 datasheet p46)
 *
 *  @param value: 8 byte value with *Least Significant Bit* first *
 *
 */
void nRF24_RegWrite(struct nRF24_Handle nRF24_H ,uint8_t address, uint8_t value)
{
	uint8_t W_address = address + 0x20;	// Add 32 to register address to write
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &W_address, 1, 1);
	HAL_SPI_Transmit(nRF24_H.hspi, &value, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// deselect slave
};




/*
 *  @brief return value in the register address
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *  @param address: the register address (please refer to nRF24L01 datasheet p46)
 *
 */
uint8_t nRF24_RegRead(struct nRF24_Handle nRF24_H, uint8_t address)
{
	uint8_t data;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &address, 1, 1);
	HAL_SPI_Receive(nRF24_H.hspi, &data, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// deselect slave
	return data;
};







/*
 *  @brief Flush TX FIFO, used in TX mode
 */
void nRF24_FlushTX(struct nRF24_Handle nRF24_H)
{
	uint8_t Flush_TX = 225;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &Flush_TX, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// deselect slave
};

/*
 *  @brief Flush RX FIFO, used in RX mode.
 *         (Should not be used during transmission of acknowledge)
 */
void nRF24_FlushRX(struct nRF24_Handle nRF24_H)
{
	uint8_t Flush_RX = 226;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &Flush_RX, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// deselect slave
};

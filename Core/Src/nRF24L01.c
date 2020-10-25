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
 *  @brief Write value into the register address
 *
 *  @param address: the register address (please refer to nRF24L01 datasheet p46)
 *
 *  @param value: 8 byte value with *Least Significant Bit* first *
 *
 */
void reg_write(uint8_t address, uint8_t value)
{
	uint8_t W_address = address + 0x20;	// Add 32 to register address to write
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(&hspi1, &W_address, 1, 1);
	HAL_SPI_Transmit(&hspi1, &value, 1, 1);
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);	// deselect slave
};

/*
 *  @brief return value in the register address
 *
 *  @param address: the register address (please refer to nRF24L01 datasheet p46)
 *
 */
uint8_t reg_read(uint8_t address)
{
	uint8_t data;
 	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
	HAL_SPI_Transmit(&hspi1, &address, 1, 1);
	HAL_SPI_Receive(&hspi1, &data, 1, 1);
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);	// deselect slave
	return data;
};

/*
 *  @brief read value in the pipe address
 *
 *  @param pipe: data pipe, from 0 to 6
 *
 */
void read_Pipe_address(uint8_t pipe)
{
	uint8_t pipe_buffer[5];
	uint8_t pipe_address = pipe + 0x0A;
 	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
 	HAL_SPI_Transmit(&hspi1, &pipe_address, 1, 1);
 	HAL_SPI_Receive(&hspi1, pipe_buffer, 5, 10);
 	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);	// deselect slave
	CDC_Transmit_FS(pipe_buffer, sizeof(pipe_buffer));
};

/*
 *  @brief Flush TX FIFO, used in TX mode
 */
void flush_TX()
{
	uint8_t Flush_TX = 225;
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
	HAL_SPI_Transmit(&hspi1, &Flush_TX, 1, 1);
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);	// deselect slave
};

/*
 *  @brief Flush RX FIFO, used in RX mode.
 *         (Should not be used during transmission of acknowledge)
 */
void flush_RX()
{
	uint8_t Flush_RX = 226;
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
	HAL_SPI_Transmit(&hspi1, &Flush_RX, 1, 1);
	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);	// deselect slave
};

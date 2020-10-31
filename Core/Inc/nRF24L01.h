/*
 * nRF24L01.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Morgan
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h"








// TODO: Add data transfer function

struct nRF24_Handle
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *CSN_GPIO_Port;	// Slave select OUTPUT pin (Active High)
	uint16_t CSN_Pin;
	GPIO_TypeDef *CE_GPIO_Port;		// OUTPUT Pin that activate TX, RX or Standby II mode (Active High)
	uint16_t CE_Pin;
	GPIO_TypeDef *IQR_GPIO_Port;	// Interrupt PULL UP INPUT PIN (Active Low)
	uint16_t IQR_Pin;
};


// Quick Start Functions
void nRF24_QS(struct nRF24_Handle nRF24_H, uint8_t TX_mode);
void nRF24_QSconfig(struct nRF24_Handle nRF24_H);

// High Level Function

void nRF24_TX_WritePayload(struct nRF24_Handle nRF24_H, uint8_t* payload, uint8_t payload_size, uint8_t write_type);
void nRF24_TX_SendPayload(struct nRF24_Handle nRF24_H, uint8_t send_all);

uint8_t nRF24_RX_DataAvaliable(struct nRF24_Handle nRF24_H);
void nRF24_RX_ReadPayload(struct nRF24_Handle nRF24_H, uint8_t* rx_buffer, uint8_t buffer_size);
uint8_t nRF24_RX_GetPayloadPipe(struct nRF24_Handle nRF24_H);
uint8_t nRF24_RX_DataInPipe(struct nRF24_Handle nRF24_H);

// Config
void nRF24_SetDataPipeADDR(struct nRF24_Handle nRF24_H, uint8_t reg_addr, uint8_t* pipe_addr);
void nRF24_GetDataPipeADDR(struct nRF24_Handle nRF24_H, uint8_t reg_addr, uint8_t* pipe_addr);

void nRF24_SetDataPipeSize(struct nRF24_Handle nRF24_H, uint8_t pipe_addr, uint8_t pipe_size);
uint8_t nRF24_GetDataPipeSize(struct nRF24_Handle nRF24_H, uint8_t pipe_addr);


// TODO: Finish the following functions
void nRF24_ConnectionCheck(struct nRF24_Handle nRF24_H);


// Low Level Functions

void nRF24_RegWrite(struct nRF24_Handle nRF24_H, uint8_t address, uint8_t value);
uint8_t nRF24_RegRead(struct nRF24_Handle nRF24_H, uint8_t address);
void nRF24_FlushTX(struct nRF24_Handle nRF24_H);
void nRF24_FlushRX(struct nRF24_Handle nRF24_H);


#endif /* INC_NRF24L01_H_ */

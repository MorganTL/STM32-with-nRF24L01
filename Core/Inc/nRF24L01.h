/*
 * nRF24L01.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Morgan
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h"


#define p0size_addr = 0x11
#define p1size_addr = 0x12
#define p2size_addr = 0x13
#define p3size_addr = 0x14
#define p4size_addr = 0x15
#define p5size_addr= 0x16






// TODO: Add data transfer function

struct nRF24_Handle
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *CSN_GPIO_Port;
	uint16_t CSN_Pin;
	GPIO_TypeDef *CE_GPIO_Port;
	uint16_t CE_Pin;
};


// Quick Start Functions
void nRF24_QS(struct nRF24_Handle nRF24_H);
void nRF24_QSconfig(struct nRF24_Handle nRF24_H);

// High Level Function
void nRF24_SetXXX();
void nRF24_SetDataPipeSize(struct nRF24_Handle nRF24_H, uint8_t pipe_addr, uint8_t size);

int nRF24_GetPipeSize(struct nRF24_Handle nRF24_H, uint8_t pipe_addr);

void nRF24_TX_LoadPayload(struct nRF24_Handle nRF24_H);
void nRF24_TX_SendPayload(struct nRF24_Handle nRF24_H, uint8_t byte);
int nRF24_Avaliable(struct nRF24_Handle nRF24_H);
void nRF24_RX_ReadPipe(struct nRF24_Handle nRF24_H, uint8_t pipe_addr, uint8_t* rx_buffer, uint8_t buffer_size);


// Low Level Functions
void nRF24_RegWrite(struct nRF24_Handle nRF24_H, uint8_t address, uint8_t value);
uint8_t nRF24_RegRead(struct nRF24_Handle nRF24_H, uint8_t address);




void nRF24_FlushTX(struct nRF24_Handle nRF24_H);
void nRF24_FlushRX(struct nRF24_Handle nRF24_H);


#endif /* INC_NRF24L01_H_ */

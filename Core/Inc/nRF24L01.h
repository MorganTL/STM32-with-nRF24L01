/*
 * nRF24L01.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Morgan
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h"


// Basic functions
void reg_write(uint8_t address, uint8_t value);
uint8_t reg_read(uint8_t address);
void read_Pipe_address(uint8_t pipe);


void flush_TX();
void flush_RX();


#endif /* INC_NRF24L01_H_ */

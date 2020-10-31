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

// SPI Commands
uint8_t W_TX_Payload = 0xA0;
uint8_t R_RX_Payload = 0x61;
uint8_t R_RX_PL_WID = 0x60;

// Register Addresses
const uint8_t CONFIG = 0x00;
const uint8_t EN_AA = 0x01;		// Enhance ShockBurst^TM
const uint8_t EN_RXADDR = 0x02;		//Enable RX data pipe
const uint8_t SETUP_AW = 0x03;		//Data Pipe address width 0-5 byte
const uint8_t SETUP_RETR = 0x04;	//Auto Retransmission
const uint8_t RF_CH = 0x05;
const uint8_t RF_SETUP = 0x06;
const uint8_t STATUS = 0x07; // Register Address for interrupt PIN IQR
const uint8_t p0size_addr = 0x11;
const uint8_t p1size_addr = 0x12;
const uint8_t p2size_addr = 0x13;
const uint8_t p3size_addr = 0x14;
const uint8_t p4size_addr = 0x15;
const uint8_t p5size_addr = 0x16;
const uint8_t FIFO_STATUS = 0x17;
const uint8_t FEATURE = 0x1D;



/*
 *  @brief Power on and set up all the necessary configs for fast prototyping
 */
void nRF24_QS(struct nRF24_Handle nRF24_H, uint8_t TX_mode)
{
	HAL_Delay(100);		// 100ms Power on time
	HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_RESET);	// Standby I mode
	HAL_Delay(20);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);
	nRF24_QSconfig(nRF24_H);

	nRF24_RegWrite(nRF24_H, EN_RXADDR, 0x03);	// Open Port 0,1
	nRF24_RegWrite(nRF24_H, p0size_addr, 0x02);
	nRF24_RegWrite(nRF24_H, p1size_addr, 0x01);

	nRF24_FlushTX(nRF24_H);
	if(!TX_mode)
	{
		nRF24_RegWrite(nRF24_H, CONFIG, 0x0B); // PRX, Power ON, CRC, 1 byte
		nRF24_RegWrite(nRF24_H, EN_RXADDR, 0x02);	// Open Port 0 and close 1

		nRF24_FlushRX(nRF24_H);

		HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_SET);	// RX mode, Start listening
		HAL_Delay(2);		// 130us delay before listening
	}

	nRF24_RegWrite(nRF24_H, STATUS, 0x70);	// Clear TX/RX Interrupt
//	HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_SET);	// RX or TX mode


};

/*
 *  @brief Set up all the necessary configs for fast prototyping
 */
void nRF24_QSconfig(struct nRF24_Handle nRF24_H)
{
	//nRF24_RegWrite(nRF24_H, CONFIG, 0x0A); // PTX, Power ON, CRC, 1 byte
	nRF24_RegWrite(nRF24_H, CONFIG, 0x0A); // PTX, Power ON, CRC, 1 byte
	nRF24_RegWrite(nRF24_H, SETUP_AW, 3);	// 5 byte channel address width
	nRF24_RegWrite(nRF24_H, SETUP_RETR, 0xFF); // 15 Retry, 0.25 ms wait (0x0F -> 0xFF 4ms wait)
	nRF24_RegWrite(nRF24_H, RF_CH, 0x20); // 2400 + 0x11 Mhz
	nRF24_RegWrite(nRF24_H, RF_SETUP, 0x0E); // 0dBm, 2Mbp (0x0E)

	nRF24_RegWrite(nRF24_H, FEATURE, 0x00);		// Dynamic payload length + Payload with ARK 0x06 (Default = 0x00)

	for (uint8_t i_addr = 0x11; i_addr < 0x17 ; i_addr++)	// Close all port
	{
		nRF24_RegWrite(nRF24_H, i_addr, 0x00);
	}
};

/*
 *  @brief Load Payload into TX FIFO. If TX FIFO contains more than one payloads,
 *		   the payloads are handled in a First in, First out principle.
 *
 * 	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *  @param payload: a 1 to 32 byte data packet
 *
 *  @param payload_size: 1 to 32 byte
 *
 *  @param write_type: 0xA0 (w/ ARK) or 0xB0 (w/o ARK)
 *
 */
void nRF24_TX_WritePayload(struct nRF24_Handle nRF24_H, uint8_t* payload, uint8_t payload_size, uint8_t write_type)
{
	if (write_type != 0xA0 || write_type != 0xB0)
	{
		// TODO: Raise Error
	}

	uint8_t TX_full = (nRF24_RegRead(nRF24_H, FIFO_STATUS) & 0x20) >> 5;	// Get TX FIFO Full flag

	if (sizeof(payload) < 33 && !TX_full)
	{
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &write_type, sizeof(write_type), 1);		// Write with or w/o ART (DO NOT CHANGE THIS!!)
	HAL_SPI_Transmit(nRF24_H.hspi, payload, payload_size, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
	}
	else
	{
		// TODO: Raise Error
	}


};
/*
 *	TODO: Fill in comments
 */
void nRF24_TX_SendPayload(struct nRF24_Handle nRF24_H, uint8_t send_all)
{

	uint8_t paydload_empty = ((nRF24_RegRead(nRF24_H, FIFO_STATUS) & 0x10) >> 4); // Get the TX FIFO empty flag
	if(!send_all && !paydload_empty)
	{
		HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_SET);
		HAL_Delay(20);	// TODO: Find prefect time!!
//		nRF24_RegWrite(nRF24_H, STATUS, 0x70);	// Clear TX/RX Interrupt
	}
	else if(!paydload_empty)
	{
		// TODO: send all packets
	}
	else
	{
		// TODO: payload empty ERROR
	}

	uint8_t TX_DS, MAX_RT;
	do	// Check whether the packet is sent or lock in transmission
	{
		TX_DS = ((nRF24_RegRead(nRF24_H, STATUS) & 0x20) >> 0x05);
		MAX_RT = ((nRF24_RegRead(nRF24_H, STATUS) & 0x10) >> 0x04);
	} while(!TX_DS && !MAX_RT);

	HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_RESET);

};



/*
 *	TODO: FIll in comments
 */
uint8_t nRF24_RX_DataAvaliable(struct nRF24_Handle nRF24_H)
{
	return (HAL_GPIO_ReadPin(nRF24_H.IQR_GPIO_Port, nRF24_H.IQR_Pin));
};

/*
 *	TODO: FIll in comments
 */
void nRF24_RX_ReadPayload(struct nRF24_Handle nRF24_H, uint8_t* rx_buffer, uint8_t buffer_size)
{
	uint8_t RX_Empty = (nRF24_RegRead(nRF24_H, 0x17) & 0x01);	// Data in FIFO
	uint8_t R_RX = 0x60;
	if(!RX_Empty)
	{
		HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_RESET);	// Stop listening
		HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
		HAL_SPI_Transmit(nRF24_H.hspi, &(R_RX_Payload), sizeof(R_RX_Payload), 1);
		HAL_SPI_Receive(nRF24_H.hspi, rx_buffer, buffer_size, 20);
		HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
		HAL_GPIO_WritePin(nRF24_H.CE_GPIO_Port, nRF24_H.CE_Pin, GPIO_PIN_SET);	// Start listening
		nRF24_RegWrite(nRF24_H, 0x07, 0x70);		// Clear interrupt
	}
};

/*
 *	TODO: Fill in comments
 */
uint8_t nRF24_GetPayloadPipe(struct nRF24_Handle nRF24_H)
{
	uint8_t pipe_no = nRF24_RegRead(nRF24_H, 0x17);
	pipe_no = ( (pipe_no & 0x0E) >> 1);
	return pipe_no;
};

/*
 *
 */
uint8_t nRF24_RX_DataInPipe(struct nRF24_Handle nRF24_H)
{
	uint8_t Temp = nRF24_RegRead(nRF24_H, 0x17);
	return (Temp & 0x01);
}





/*
 *	@brief Write the 5 byte data pipe identify address to reg_addr
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *	@param reg_addr: data pipe register address (from 0x0A to 0x10, see datasheet for details)
 *
 *	@param pipe_addr: the pipe identify address
 *
 */
void nRF24_SetDataPipeADDR(struct nRF24_Handle nRF24_H, uint8_t reg_addr, uint8_t* pipe_addr)
{
	if (sizeof(pipe_addr) > 3)
	{
		uint8_t W_address = reg_addr + 0x20;	// Add 32 to register address to write
		HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
		HAL_SPI_Transmit(nRF24_H.hspi, &(W_address), 1, 1);
		HAL_SPI_Receive(nRF24_H.hspi, pipe_addr, 5, 20);
		HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
	}
	else
	{
		// TODO: Raise ERROR
	}
};

/*
 *	@brief Read the 5 byte data pipe identify address
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *	@param reg_addr: data pipe register address (from 0x0A to 0x10, see datasheet for details)
 *
 *	@param pipe_addr: a uint8_t pointer with at least 5 element (output)
 *
 */
void nRF24_GetDataPipeADDR(struct nRF24_Handle nRF24_H, uint8_t reg_addr, uint8_t* pipe_addr)
{
	// TODO: add a array size check
	if(pipe_addr)
	{
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &(reg_addr), 1, 1);
	HAL_SPI_Receive(nRF24_H.hspi, pipe_addr, 5, 10);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
	}
	else
	{
		// TODO: Raise ERROR (array size is too small)
	}
};

/*
 *	@brief Change the payload size of the data pipe (used in RX mode)
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info
 *
 *	@param pipe_addr: from 0x11 to 0x16 (TODO: change this to enum)
 *
 *	@param pipe_size: 0 - 32 bytes (0: Pipe is closed)
 *
 */
void nRF24_SetDataPipeSize(struct nRF24_Handle nRF24_H, uint8_t pipe_addr, uint8_t pipe_size)
{
	nRF24_RegWrite(nRF24_H, pipe_size, pipe_size);
}



/*
 *  @brief Write value into the register address
 *
 * 	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *  @param address: the register address (please refer to nRF24L01 datasheet p46)
 *
 *  @param value: 8 byte value, *Least Significant Bit* first
 *
 */
void nRF24_RegWrite(struct nRF24_Handle nRF24_H ,uint8_t address, uint8_t value)
{
	uint8_t W_address = address + 0x20;	// Add 32 to register address to write
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &W_address, 1, 1);
	HAL_SPI_Transmit(nRF24_H.hspi, &value, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
};




/*
 *  @brief return byte in the register address / from SPI command
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 *  @param address: the register address or SPI command (please refer to nRF24L01 datasheet p46)
 *
 */
uint8_t nRF24_RegRead(struct nRF24_Handle nRF24_H, uint8_t address)
{
	uint8_t data;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &address, 1, 1);
	HAL_SPI_Receive(nRF24_H.hspi, &data, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
	return data;
};


/*
 *  @brief Flush TX FIFO, used in TX mode
 *
 *  @param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 */
void nRF24_FlushTX(struct nRF24_Handle nRF24_H)
{
	uint8_t Flush_TX = 225;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &Flush_TX, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
};

/*
 *  @brief Flush RX FIFO, used in RX mode.
 *         (Should not be used during transmission of acknowledge)
 *
 *	@param nRF24_H: nRF24 handler which contains all necessary pins info.
 *
 */
void nRF24_FlushRX(struct nRF24_Handle nRF24_H)
{
	uint8_t Flush_RX = 226;
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_RESET);	// Select slave
	HAL_SPI_Transmit(nRF24_H.hspi, &Flush_RX, 1, 1);
	HAL_GPIO_WritePin(nRF24_H.CSN_GPIO_Port, nRF24_H.CSN_Pin, GPIO_PIN_SET);	// Deselect slave
};

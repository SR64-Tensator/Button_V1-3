#ifndef __NRF24_H
#define __NRF24_H

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#ifndef nRF24_ADDR_REVERSE
// How the TX/RX address should be transmitted to the transceiver
//   0 - straight - the first byte of the address transmitted first
//   1 - reverse - the last byte of the address transmitted first
#define nRF24_ADDR_REVERSE         0
#endif


// nRF24L0 instruction definitions
#define nRF24_CMD_R_REGISTER       (uint8_t)0x00 // Register read
#define nRF24_CMD_W_REGISTER       (uint8_t)0x20 // Register write
#define nRF24_CMD_R_RX_PAYLOAD     (uint8_t)0x61 // Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     (uint8_t)0xA0 // Write TX payload
#define nRF24_CMD_FLUSH_TX         (uint8_t)0xE1 // Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         (uint8_t)0xE2 // Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      (uint8_t)0xE3 // Reuse TX payload
#define nRF24_CMD_LOCK_UNLOCK      (uint8_t)0x50 // Lock/unlock exclusive features
#define nRF24_CMD_NOP              (uint8_t)0xFF // No operation (used for reading status register)

// nRF24L0 register definitions
#define nRF24_REG_CONFIG           (uint8_t)0x00 // Configuration register
#define nRF24_REG_EN_AA            (uint8_t)0x01 // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        (uint8_t)0x02 // Enable RX addresses
#define nRF24_REG_SETUP_AW         (uint8_t)0x03 // Setup of address widths
#define nRF24_REG_SETUP_RETR       (uint8_t)0x04 // Setup of automatic retransmit
#define nRF24_REG_RF_CH            (uint8_t)0x05 // RF channel
#define nRF24_REG_RF_SETUP         (uint8_t)0x06 // RF setup register
#define nRF24_REG_STATUS           (uint8_t)0x07 // Status register
#define nRF24_REG_OBSERVE_TX       (uint8_t)0x08 // Transmit observe register
#define nRF24_REG_RPD              (uint8_t)0x09 // Received power detector
#define nRF24_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          (uint8_t)0x10 // Transmit address
#define nRF24_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
#define nRF24_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
#define nRF24_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
#define nRF24_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
#define nRF24_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
#define nRF24_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5
#define nRF24_REG_FIFO_STATUS      (uint8_t)0x17 // FIFO status register
#define nRF24_REG_DYNPD            (uint8_t)0x1C // Enable dynamic payload length
#define nRF24_REG_FEATURE          (uint8_t)0x1D // Feature register

// Retransmit delay
enum {
	nRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	nRF24_ARD_250us  = (uint8_t)0x00,
	nRF24_ARD_500us  = (uint8_t)0x01,
	nRF24_ARD_750us  = (uint8_t)0x02,
	nRF24_ARD_1000us = (uint8_t)0x03,
	nRF24_ARD_1250us = (uint8_t)0x04,
	nRF24_ARD_1500us = (uint8_t)0x05,
	nRF24_ARD_1750us = (uint8_t)0x06,
	nRF24_ARD_2000us = (uint8_t)0x07,
	nRF24_ARD_2250us = (uint8_t)0x08,
	nRF24_ARD_2500us = (uint8_t)0x09,
	nRF24_ARD_2750us = (uint8_t)0x0A,
	nRF24_ARD_3000us = (uint8_t)0x0B,
	nRF24_ARD_3250us = (uint8_t)0x0C,
	nRF24_ARD_3500us = (uint8_t)0x0D,
	nRF24_ARD_3750us = (uint8_t)0x0E,
	nRF24_ARD_4000us = (uint8_t)0x0F
};

// Data rate
enum {
	nRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	nRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	nRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
};

// RF output power in TX mode
enum {
	nRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	nRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	nRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	nRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
};

// CRC encoding scheme
enum {
	nRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
	nRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	nRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
};

// nRF24L01 power control
enum {
	nRF24_PWR_UP   = (uint8_t)0x02, // Power up
	nRF24_PWR_DOWN = (uint8_t)0x00  // Power down
};

// Transceiver mode
enum {
	nRF24_MODE_RX = (uint8_t)0x01, // PRX
	nRF24_MODE_TX = (uint8_t)0x00  // PTX
};

// Enumeration of RX pipe addresses and TX address
enum {
	nRF24_PIPE0  = (uint8_t)0x00, // pipe0
	nRF24_PIPE1  = (uint8_t)0x01, // pipe1
	nRF24_PIPE2  = (uint8_t)0x02, // pipe2
	nRF24_PIPE3  = (uint8_t)0x03, // pipe3
	nRF24_PIPE4  = (uint8_t)0x04, // pipe4
	nRF24_PIPE5  = (uint8_t)0x05, // pipe5
	nRF24_PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
};

// State of auto acknowledgment for specified pipe
enum {
	nRF24_AA_OFF = (uint8_t)0x00,
	nRF24_AA_ON  = (uint8_t)0x01
};

// Status of the RX FIFO
enum {
	nRF24_STATUS_RXFIFO_DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
	nRF24_STATUS_RXFIFO_EMPTY = (uint8_t)0x01, // The RX FIFO is empty
	nRF24_STATUS_RXFIFO_FULL  = (uint8_t)0x02, // The RX FIFO is full
	nRF24_STATUS_RXFIFO_ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
};

// Status of the TX FIFO
enum {
	nRF24_STATUS_TXFIFO_DATA  = (uint8_t)0x00, // The TX FIFO contains data and available locations
	nRF24_STATUS_TXFIFO_EMPTY = (uint8_t)0x01, // The TX FIFO is empty
	nRF24_STATUS_TXFIFO_FULL  = (uint8_t)0x02, // The TX FIFO is full
	nRF24_STATUS_TXFIFO_ERROR = (uint8_t)0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
};

// Result of RX FIFO reading
typedef enum {
	nRF24_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	nRF24_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	nRF24_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	nRF24_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	nRF24_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	nRF24_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	nRF24_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} nRF24_RXResult;

enum {
	Receiver_Mode_AA            = (uint8_t)0x00,    // Receiver mode with Auto Acknowledgment enabled
	Receiver_Mode_noAA_CRC      = (uint8_t)0x01,    // Receiver mode without Auto Acknowledgment and with CRC enabled
	Receiver_Mode_noAA_noCRC    = (uint8_t)0x02,    // Receiver mode without Auto Acknowledgment and CRC
	Transmitter_Mode_AA         = (uint8_t)0x03,    // Transmitter mode with Auto Acknowledgment enabled
	Transmitter_Mode_noAA_CRC   = (uint8_t)0x04,    // Transmitter mode without Auto Acknowledgment and with CRC enabled
	Transmitter_Mode_noAA_noCRC = (uint8_t)0x05     // Transmitter mode without Auto Acknowledgment and CRC
};

enum {
	MBPS1_Minus18dBm    = (uint8_t)0x00,            //-18dBm, 1MBPS, LNA_HCURR=1
	MBPS1_Minus12dBm    = (uint8_t)0x01,            //-12dBm, 1MBPS, LNA_HCURR=1
	MBPS1_Minus6dBm     = (uint8_t)0x02,            //-6dBm, 1MBPS, LNA_HCURR=1
	MBPS1_0dBm          = (uint8_t)0x03,            //0dBm, 1MBPS, LNA_HCURR=1
	MBPS2_Minus18dBm    = (uint8_t)0x04,            //-18dBm, 2MBPS, LNA_HCURR=1
	MBPS2_Minus12dBm    = (uint8_t)0x05,            //-12dBm, 2MBPS, LNA_HCURR=1
	MBPS2_Minus6dBm     = (uint8_t)0x06,            //-6dBm, 2MBPS, LNA_HCURR=1
	MBPS2_0dBm          = (uint8_t)0x07             //0dBm, 2MBPS, LNA_HCURR=1
};

enum {
	Power_Down    = (uint8_t)0x00,            //-18dBm, 1MBPS, LNA_HCURR=1
	Power_Up    = (uint8_t)0x01,            //-12dBm, 1MBPS, LNA_HCURR=1
};

// Function prototypes

void nRF24_CSN_Select(uint16_t Sel_CSN_Pin);
void nRF24_CSN_UnSelect(uint16_t Sel_CSN_Pin);
void nRF24_CE_Enable(uint16_t Sel_CE_Pin);
void nRF24_CE_Disable(uint16_t Sel_CE_Pin);
void nRF24_WriteReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t data);
void nRF24_WriteMBReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t *data, uint8_t size);
uint8_t nRF24_ReadReg(uint16_t Sel_CSN_Pin, uint8_t reg);
void nRF24_ReadMBReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t *data, uint8_t size);
void nRF24_SendCMD(uint16_t Sel_CSN_Pin, uint8_t cmd);
void nRF24_Reset(uint16_t Sel_CSN_Pin, uint8_t Reg);
uint8_t nRF24_Transmit(uint16_t Sel_CSN_Pin, uint8_t *data, uint8_t Payload_Size);
uint8_t isDataAvailable(uint16_t Sel_CSN_Pin);
void nRF24_Receive(uint16_t Sel_CSN_Pin, uint8_t *data, uint8_t Payload_Size);
void nRF24_Module_Setup(uint16_t Sel_CSN_Pin, uint8_t mode, uint8_t RF_Ch, uint8_t *Tx_Addr, uint8_t *Rx_Addr, uint8_t RF_Setup, uint8_t Payloadsize);
void nRF24_ReadAllRegs (uint16_t Sel_CSN_Pin, uint8_t *data);
void nRF24_Power(uint16_t Sel_CSN_Pin, uint8_t On_Off_Cmd);
void nRF24_Transmit_Report(uint16_t Sel_CSN_Pin);
void nRF24_Register_Display(uint16_t Sel_CSN_Pin);
void nRF24_Receive_Report(uint16_t Sel_CSN_Pin);
void nRF24_sendRegisterValue(uint8_t value);

#endif // __NRF24_H

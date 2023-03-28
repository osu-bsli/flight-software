/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
//#include <avr/io.h>
//#include <util/delay.h>
//#include <avr/interrupt.h>
//#include <util/atomic.h>
//#include "mcc_generated_files/mcc.h"
//#include "CAN_ADDRESS_DEFINITION.h"
//#include "main.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
// Define unsigned 8 bit integer
typedef unsigned char uint_8;


// Define "logic" for CS function
//typedef char bool;
#define HIGH 0x01
#define LOW 0x00

#define CAN_RESET 0xC0
#define CAN_READ 0x03
#define CAN_WRITE 0x02
#define CAN_BIT_MODIFY 0x05

// Can be any 0xnF ?
#define CANCTRL 0x0F
#define BFPCTRL 0x0C

#define CANINTF 0x2C
#define CANINTE 0x2B

// Mask and filter addresses
// Mask for buffer 0
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
// mask for buffer 1
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
// filters (just low and high since we dont use extended)
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19


// CAN configuration registers
#define CNF1 0x2A
#define CNF2 0x29
#define CNF3 0x28

// CAN Status register
#define CANSTAT 0x0E

// 3 Buffers each has 8 bytes but all sequential so as long as
// we know the address of the first one
#define TXB0D0 0x36
#define TXB1D0 0x46
#define TXB2D0 0x56

#define LOW_PRIORITY 0x00
#define MED_LOW_PRIORITY 0x01
#define MED_HIGH_PRIORITY 0x02
#define HIGH_PRIORITY 0x03

// Receive control register addresses
#define RXB0CTRL 0x60
#define RXB1CTRL 0x70

// 2 buffers with multiple bytes but just reference the first one
#define RXB0D0 0x66
#define RXB1D0 0x76


// Define struct for a CAN message
// no need to set from_addr when transmitting, as it will be overwritten using
// MY_CAN_ADDR
struct CAN_message {
        char from_addr;
        char dest_addr;
        char type;
        unsigned char data_len;	// uint_8
        char data[8];
};


/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */


/**
 * Send messages to the CAN via SPI to set it up
 */
void configureCAN(void);

// Send reset over SPI
void resetCAN(void);

// Write data d to a CAN register addr
void writeCAN(char addr, char d);

// Read data from a CAN register addr
char readCAN(char addr);

// Transmit bytes over CAN bus
void CAN_Transmit(char transmit_buffer, struct CAN_message to_send, char priority);

// Read bytes that came over CAN bus
struct CAN_message CAN_Receive(char receive_buffer);

// Read a single byte from the first recieve register
char CAN_simpleRead();

// Sends a single byte with low priority from the first transmit buffer
void CAN_simpleSend(char d);

// Put CAN controller in sleep mode and transciever in STBY mode
void CAN_sleep();

// Modify bit inside can controller register
void bitModifyCAN(char addr, char bits, char d);

// Function that hangs untill all can messages are sent
void waitToSendCAN();

// Abort all pending transmissions
void abortAll();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


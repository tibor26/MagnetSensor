/**
 * This file contains all the functions
*/

/* 

*/

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

/*
 * Global Variable and Constant definitions
 * 
 */
#include "stm32g0xx_hal.h"
#include "main.h"

typedef struct
{
    uint16_t baseTimerCnt, timerCnt, delayTimeCounter;
    uint16_t timerCnt_01ms, timerCnt_05ms, timerCnt_10ms, timerCnt_50ms;
    uint16_t timerCnt_100ms;
    uint8_t  timerFlags;
    
    uint8_t     txBufferIndex, rxBufferIndex, bytesToRx, bytesToTx;
    uint8_t     txBuffer[32], rxBuffer[16];
    uint8_t     UART2Flags;
    
    uint8_t spiRxBuffer[16], spiTxBuffer[16];
    uint8_t spiRxIndex, spiTxIndex, spiBufferSize;
    
    uint8_t mSensorFlags;

    uint8_t magnetData[6];		//xl, xh, yl, yh, zl, zh

    uint32_t adcDmaBuffer[2];		// only two channels for now
    uint16_t adcPotValue, adcVbusValue;

} globalVariableType;

extern globalVariableType gv, *pgv;

extern SPI_HandleTypeDef hspi2;

extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_adc1;

//----
void App_Init(void);	// To initialize some parameters and initiate uart receiving

//-- ADC1 Functions -------

// Channel definitions,
#define	ADC_CH_POT	0
#define	ADC_CH_VBUS	1


//-- UART functions
void UART2_RxMessageProcess(void);

void shortDelay(uint16_t time);     // delay a number of Fosc cycles. Not accurate
void myDelay(uint16_t delayTime);    // delay Time in ms. using timer1
void timedAppTasks(void);

void timedEvents_01ms(void);
void timedEvents_05ms(void);
void timedEvents_10ms(void);
void timedEvents_50ms(void);
void timedEvents_100ms(void);

//-- SPI functions
HAL_StatusTypeDef mSensor_Init(void);
HAL_StatusTypeDef mSensor_Read(void);	// To read the magnetic fields out, x, y, z total 6 bytes.
HAL_StatusTypeDef mSensor_Read_Register(uint8_t address);	//Return the register value
HAL_StatusTypeDef mSensor_Read_Block(uint8_t address, uint8_t *pdata, uint8_t length);	//To read a block of data out of the sensor
HAL_StatusTypeDef mSensor_Write_Register(uint8_t address, uint8_t data);		// To write one byte to the register

//- UART variables
#define MSG_HEADER_OUT  0xA5        // Message sent to PC
#define MSG_HEADER_IN   0x5A        // Message received 
#define MSG_TYPE_TAG    0x01
#define MSG_TYPE_SN     0x02
#define MSG_TYPE_BLOCK_DATA   0x03

#define fUART2_RxReady   0x01
#define fUART2_TxReady   0x02
#define fUART2_RxMsgReady   0x04
#define fUART2_Txing    0x08

// Timer variables
#define ftimer_01ms  0x01
#define ftimer_05ms  0x02
#define ftimer_10ms  0x04
#define ftimer_50ms  0x08
#define ftimer_100ms  0x10

// Magnet sensor variables
#define	fmsensor_ready	0x01

#define	SENSOR_ID	0x3D
#define	GAUSS_SCALE_4	0x00
#define	GAUSS_SCALE_8	0x20
#define	GAUSS_SCALE_12	0x40
#define	GAUSS_SCALE_16	0x60



#endif //FUNCTIONS_H

/**
 End of File
*/



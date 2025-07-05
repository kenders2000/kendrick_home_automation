#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_
/***********************************************************************************************************************
			------------------------------------------------------------------------
			|\\\																///|
			|\\\					Hardware interface							///|
			------------------------------------------------------------------------
***********************************************************************************************************************/

#include "rpi_gpiod.h"
#include "dev_hardware_SPI.h"
#include "dev_hardware_i2c.h"
#include "dev_hardware_UART.h"

#include <stdint.h>
#include "Debug.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define DEV_SPI 0
#define DEV_I2C 0
#define DEV_UART 1

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/*-------------------------------------------------*/

char* get_raspberry_pi_model();

uint8_t DEV_ModuleInit(void);
void    DEV_ModuleExit(void);

void DEV_GPIO_Mode(UWORD Pin, UWORD Mode);
void DEV_Digital_Write(UWORD Pin, uint8_t Value);
uint8_t DEV_Digital_Read(UWORD Pin);

void DEV_Delay_ms(UDOUBLE xms);

#if DEV_I2C
    void DEV_I2C_Init(uint8_t Add);
    void I2C_Write_Byte(uint8_t Cmd, uint8_t value);
    int I2C_Read_Byte(uint8_t Cmd);
    int I2C_Read_Word(uint8_t Cmd);
    void I2C_Read_Nbyte(uint8_t Cmd,uint8_t *pdata,uint8_t len);
#endif

#if DEV_SPI
    void DEV_SPI_WriteByte(uint8_t Value);
    void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len);
#endif

#if DEV_UART
    void DEV_UART_Init(char *Device,uint32_t baud);
    void UART_Write_Byte(uint8_t data);
    int UART_Read_Byte(void);
    void UART_Set_Baudrate(uint32_t Baudrate);
    void UART_Write_nByte(uint8_t *pData, uint32_t Lan);
    void UART_Read_nByte(uint8_t *pData, uint32_t Lan);
    void UART_flushInput();
#endif

#endif

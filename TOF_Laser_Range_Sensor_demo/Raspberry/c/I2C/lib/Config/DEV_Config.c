/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2019-07-08
* | Info        :   Basic version
*
******************************************************************************/
#include "DEV_Config.h"
#include <unistd.h>
#include <fcntl.h>

uint32_t fd,UART_fd;
char *UART_Device;
/******************************************************************************
function:	Equipment Testing
parameter:
Info:   Only supports Jetson Nano and Raspberry Pi
******************************************************************************/
static int DEV_Equipment_Testing(void)
{
	int i;
	int fd;
	char value_str[20];
	fd = open("/etc/issue", O_RDONLY);
    printf("Current environment: ");
	while(1) {
		if (fd < 0) {
			return -1;
		}
		for(i=0;; i++) {
			if (read(fd, &value_str[i], 1) < 0) {
				return -1;
			}
			if(value_str[i] ==32) {
				printf("\r\n");
				break;
			}
			printf("%c",value_str[i]);
		}
		break;
	}

	if(i<5) {
		printf("Unrecognizable\r\n");
        return -1;
	} else {
		char RPI_System[10]   = {"Debian"};
		for(i=0; i<6; i++) {
			if(RPI_System[i] != value_str[i]) {
                return 'J';
			}
		}
        return 'R';
	}
	return -1;
}


/******************************************************************************
function:	GPIO Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_GPIO_Mode(uint16_t Pin, uint16_t Mode)
{
    /*
        0:  INPT   
        1:  OUTP
    */
    if(Mode == 0 || Mode == GPIOD_IN) {
        GPIOD_Direction(Pin, GPIOD_IN);
        // Debug("IN Pin = %d\r\n",Pin);
    } else {
        GPIOD_Direction(Pin, GPIOD_OUT);
        // Debug("OUT Pin = %d\r\n",Pin);
    }
}

void DEV_Digital_Write(uint16_t Pin, uint8_t Value)
{
    GPIOD_Write(Pin, Value);
}

uint8_t DEV_Digital_Read(uint16_t Pin)
{
    uint8_t Read_value = 0;
    Read_value = GPIOD_Read(Pin);
    return Read_value;
}


/**
 * delay x ms
**/
void DEV_Delay_ms(UDOUBLE xms)
{
    UDOUBLE i;
    for(i=0; i < xms; i++){
        usleep(1000);
    }
}


void GPIO_Config(void)
{
    int Equipment = DEV_Equipment_Testing();
    if(Equipment=='R'){
        /************************
        Raspberry Pi GPIO
        ***********************/
        GPIOD_Export();
        
    }else if(Equipment=='J'){
        /************************
        Jetson Nano GPIO
        ***********************/
        

    }else{
        printf("Device read failed or unrecognized!!!\r\n");
        while(1);
    }
    
}

/******************************************************************************
function:	SPI Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_SPI_Init()
{
    printf("DEV SPI Device\r\n"); 
    DEV_HARDWARE_SPI_begin("/dev/spidev0.0");
}

void DEV_SPI_WriteByte(uint8_t Value)
{
#if DEV_SPI
        DEV_HARDWARE_SPI_TransferByte(Value);
#endif
}

void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len)
{
#if DEV_SPI   
        DEV_HARDWARE_SPI_Transfer(pData, Len);  
#endif
}
/******************************************************************************
function:	I2C Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_I2C_Init(uint8_t Add)
{
#if DEV_I2C
        printf("DEV I2C Device\r\n"); 
        DEV_HARDWARE_I2C_begin("/dev/i2c-1");
        DEV_HARDWARE_I2C_setSlaveAddress(Add);
#endif
}

void DEV_I2C_Set_Slave_Addr(uint8_t Add)
{
#if DEV_I2C
        DEV_HARDWARE_I2C_setSlaveAddress(Add);
#endif
}

void I2C_Write_Byte(uint8_t Cmd, uint8_t value)
{
#if DEV_I2C 
        char wbuf[2]={Cmd, value};
        DEV_HARDWARE_I2C_write(wbuf, 2);
#endif
}

int I2C_Read_Byte(uint8_t Cmd)
{
	int ref = 0;
#if DEV_I2C
        uint8_t rbuf[2]={0};
        DEV_HARDWARE_I2C_read(Cmd, rbuf, 1);
        ref = rbuf[0];
#endif
    return ref;
}

int I2C_Read_Word(uint8_t Cmd)
{
	int ref = 0;
#if DEV_I2C
    uint8_t rbuf[2] = {0};
    DEV_HARDWARE_I2C_read(Cmd, rbuf, 2);
    ref = rbuf[1]<<8 | rbuf[0];
#endif
    return ref;
}

void I2C_Read_Nbyte(uint8_t Cmd,uint8_t *pdata,uint8_t len)
{
#if DEV_I2C
    DEV_HARDWARE_I2C_read(Cmd, pdata, len);
#endif

}

/******************************************************************************
function:	SPI Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_UART_Init(char *Device,uint32_t baud)
{
    UART_Device = Device;
#if DEV_UART
    DEV_HARDWARE_UART_begin(UART_Device);
    UART_Set_Baudrate(baud);
#endif
}

void UART_Write_Byte(uint8_t data)
{
#if DEV_UART
    DEV_HARDWARE_UART_writeByte(data);
#endif
}

int UART_Read_Byte(void)
{
    int ref = 0;
#if DEV_UART
    
    ref = DEV_HARDWARE_UART_readByte();
    
#endif
    return ref;
}

void UART_Set_Baudrate(uint32_t Baudrate)
{
#if DEV_UART
        DEV_HARDWARE_UART_setBaudrate(Baudrate);
#endif
 
}

void UART_Write_nByte(uint8_t *pData, uint32_t Lan)
{
#if DEV_UART   
        DEV_HARDWARE_UART_write(pData, Lan);
#endif 
}

void UART_Read_nByte(uint8_t *pData, uint32_t Lan)
{

#if DEV_UART
    DEV_HARDWARE_UART_read(pData,Lan);
#endif

}

void UART_flushInput()
{
    DEV_HARDWARE_UART_flushInput();
}
/******************************************************************************
function:	Module Initialize, the library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
uint8_t DEV_ModuleInit(void)
{
    printf("USE_DEV_LIB \r\n");
    GPIO_Config();
    return 0;
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_ModuleExit(void)
{
#if DEV_I2C
    DEV_HARDWARE_I2C_end();
#endif
#if DEV_SPI
    DEV_HARDWARE_SPI_end();
#endif
}


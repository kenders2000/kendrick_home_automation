/*****************************************************************************
 * | File      	:   TOF_Sense.c
 * | Author      :   Waveshare team
 * | Function    :   TOF drive function
 * | Info        :
 *----------------
 * |	This version:   V1.0
 * | Date        :   2024-09-11
 * | Info        :   Basic version
 *
 ******************************************************************************/
#include "TOF_Sense.h"

TOF_Parameter TOF_0; // Define a structure to store decoded data 定义一个存放解码后数据的结构体

/******************************************************************************
function:	Get TOF data through I2C query and decode it 通过I2C查询的方式获取TOF数据，并进行解码
parameter:
Info:
******************************************************************************/
void TOF_Inquire_I2C_Decoding()
{
    uint8_t read_buf[256];
    I2C_Read_Nbyte(0x00, read_buf, TOF_REGISTER_TOTAL_SIZE); // Read all sensor data 读取传感器全部数据

    TOF_0.interface_mode = read_buf[TOF_ADDR_MODE] & 0x07; // Working mode of TOF module TOF 模块的工作模式

    TOF_0.id = read_buf[TOF_ADDR_ID]; // ID of the TOF module TOF 模块的 ID

    TOF_0.uart_baudrate = ((read_buf[TOF_ADDR_UART_BAUDRATE + 3] << 24) | (read_buf[TOF_ADDR_UART_BAUDRATE + 2] << 16) | // TOF module serial port baud rate TOF 模块的串口波特率
                           (read_buf[TOF_ADDR_UART_BAUDRATE + 1] << 8) | read_buf[TOF_ADDR_UART_BAUDRATE]);

    TOF_0.system_time = ((read_buf[TOF_ADDR_SYSTEM_TIME + 3] << 24) | (read_buf[TOF_ADDR_SYSTEM_TIME + 2] << 16) | // The time after the TOF module is powered on TOF模块上电后经过的时间
                         (read_buf[TOF_ADDR_SYSTEM_TIME + 1] << 8) | read_buf[TOF_ADDR_SYSTEM_TIME]);

    TOF_0.dis = ((read_buf[TOF_ADDR_DIS + 3] << 24) | (read_buf[TOF_ADDR_DIS + 2] << 16) | // The distance output by the TOF module TOF模块输出的距离
                 (read_buf[TOF_ADDR_DIS + 1] << 8) | read_buf[TOF_ADDR_DIS]);

    TOF_0.dis_status = ((read_buf[TOF_ADDR_DIS_STATUS]) | (read_buf[TOF_ADDR_DIS_STATUS + 1] << 8)); // Distance status indication output by TOF module TOF模块输出的距离状态指示

    TOF_0.signal_strength = ((read_buf[TOF_ADDR_SIGNAL_STRENGTH]) | (read_buf[TOF_ADDR_SIGNAL_STRENGTH + 1] << 8)); // The signal strength output by the TOF module TOF模块输出的信号强度
    TOF_0.range_precision = read_buf[TOF_ADDR_RANGE_PRECISION];                                                     // The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的

    //Print data through the terminal 通过终端打印数据
    printf("TOF id is:%d\r\n", TOF_0.id);
    printf("TOF system time is:%d ms\r\n", TOF_0.system_time);
    printf("TOF distance is:%d mm\r\n", TOF_0.dis);
    printf("TOF status is:%d\r\n", TOF_0.dis_status);
    printf("TOF signal strength is:%d\r\n", TOF_0.signal_strength);
    printf("TOF range precision is:%d\r\n\n", TOF_0.range_precision);
    if (TOF_0.interface_mode == 0) // communication interface mode,0-UART，1-CAN，2-I/O，3-IIC 通讯接口模式,0-UART，1-CAN，2-I/O，3-IIC
        printf("UART Mode.\r\n");
    else if (TOF_0.interface_mode == 1)
    {
        printf("CAN Mode.\r\n");
    }
    else if (TOF_0.interface_mode == 2)
    {
        printf("I/O Mode.\r\n");
    }
    else
        printf("I2C Mode.\r\n");

    printf("TOF uart baudrate is:%d\r\n\n", TOF_0.uart_baudrate);
}

/************************************************
function name : IIC_Change_Mode_To_UART
function function : Change the communication mode to UART mode through IIC
parameters:
return value : Whether the mark operation is correct or not, 0 is wrong, 1 is correct
*************************************************/
void IIC_Change_Mode_To_UART()
{
    I2C_Write_Byte(TOF_ADDR_MODE, IIC_CHANGE_TO_UART_DATA);
    printf("Modification successful.\r\n");
}

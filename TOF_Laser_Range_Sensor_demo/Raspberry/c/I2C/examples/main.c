#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "DEV_Config.h"
#include "TOF_Sense.h"

void  Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:Program stop\r\n"); 
    DEV_ModuleExit();
    exit(0);
}

int main(int argc, char **argv)
{
    // Exception handling:ctrl + c
    signal(SIGINT, Handler);

    DEV_I2C_Init(ADDR_SLAVE);
    while (1)
    {
        TOF_Inquire_I2C_Decoding(); //Get TOF data through I2C and decode it 通过I2C获取TOF数据，并进行解码
        // IIC_Change_Mode_To_UART(); //Switch I2C mode to UART query mode I2C模式切换成UART查询模式
        DEV_Delay_ms(20);
    }
	DEV_ModuleExit();
    return 0; 
}

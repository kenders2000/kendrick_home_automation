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
    DEV_ModuleInit();
    const char* model = get_raspberry_pi_model();
    if (strstr(model, "Raspberry Pi 5"))
        DEV_UART_Init("/dev/ttyAMA0",921600); //Open the Raspberry Pi serial port device 打开树莓派串口设备
    else
        DEV_UART_Init("/dev/ttyS0",921600); //Open the Raspberry Pi serial port device 打开树莓派串口设备

    
    while (1)
    {
        // TOF_Inquire_Decoding(0); //Query and decode TOF data 查询获取TOF数据，并进行解码
        TOF_Active_Decoding(); //Actively acquire TOF data and decode it 主动获取TOF数据，并进行解码
        DEV_Delay_ms(20);//The refresh rate defaults to 50HZ. If the refresh rate is set to 100HZ, the time here is 1/100=0.01 刷新率默认为50HZ,如果刷新率设置成100HZ,则这里的时间为1/100=0.01s
    }
	DEV_ModuleExit();
    return 0; 
}

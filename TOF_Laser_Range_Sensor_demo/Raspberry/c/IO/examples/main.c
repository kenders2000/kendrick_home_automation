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
    TOF_IO_init();
    while (1)
    {
        TOF_IO_Demo();
        DEV_Delay_ms(20);//The refresh rate defaults to 50HZ. If the refresh rate is set to 100HZ, the time here is 1/100=0.01 刷新率默认为50HZ,如果刷新率设置成100HZ,则这里的时间为1/100=0.01s
    }
	DEV_ModuleExit();
    return 0; 
}

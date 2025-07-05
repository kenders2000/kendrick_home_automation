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

void TOF_IO_init()
{
    DEV_GPIO_Mode(IO_H, GPIOD_IN);
    DEV_GPIO_Mode(IO_L, GPIOD_IN);
}

void TOF_IO_Demo()
{
    if ((DEV_Digital_Read(IO_H) == GPIOD_HIGH) && (DEV_Digital_Read(IO_L) == GPIOD_LOW))
        printf("In range.\r\n");
    else
        printf("Not in range.\r\n");
    DEV_Delay_ms(20);
}

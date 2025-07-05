#ifndef _TOF_SENSE_H_
#define _TOF_SENSE_H_

#include "DEV_Config.h"

#define TOF_FRAME_HEADER 0x57//定义TOFSense系列和TOFSense-F系列的帧头
#define TOF_FUNCTION_MARK 0x00//定义TOFSense系列和TOFSense-F系列的功能码

typedef struct {
  uint8_t id;//TOF模块的id
  uint32_t system_time;//TOF模块上电后经过的时间，单位：ms
  uint32_t dis;//TOF模块输出的距离，单位：m
  uint8_t dis_status;//TOF模块输出的距离状态指示
  uint16_t signal_strength;//TOF模块输出的信号强度
  uint8_t range_precision;//TOF模块输出的重复测距精度参考值，TOFSense-F系列有效，单位：cm
} TOF_Parameter;//解码后的TOF数据结构体

void TOF_Active_Decoding();
void TOF_Inquire_Decoding(uint8_t id);





























#endif
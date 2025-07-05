#include "TOF_Sense.h"
void setup(){
  Serial.begin(115200);
  Wire.begin(TOF_SDA,TOF_SCL);//Initialize Hardware I2C 初始化硬件I2C
}

void loop(){
  TOF_Inquire_I2C_Decoding();   //Get TOF data through I2C and decode it 通过I2C获取TOF数据，并进行解码
  // IIC_Change_Mode_To_UART(); //Switch I2C mode to UART query mode I2C模式切换成UART查询模式
  delay(20);
}
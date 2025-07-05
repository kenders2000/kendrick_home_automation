#include "TOF_Sense.h"

void setup() {
  Serial.begin(115200);//Initialize the USB serial port baud rate to 115200 初始化USB串口波特率到115200
  TOF_UART.begin(921600, SERIAL_8N1, TOF_RX_PIN, TOF_TX_PIN);//Initialize the TOF serial port baud rate to 921600 and specify the pin 初始化TOF串口波特率到921600,并指定引脚
}

void loop() {
  TOF_Active_Decoding();//Query and decode TOF data 查询获取TOF数据，并进行解码
  // TOF_Inquire_Decoding(0);//Query and decode TOF data 查询获取TOF数据，并进行解码
  delay(20);//The refresh rate defaults to 50HZ. If the refresh rate is set to 100HZ, the time here is 1/100=0.01 刷新率默认为50HZ,如果刷新率设置成100HZ,则这里的时间为1/100=0.01s
}

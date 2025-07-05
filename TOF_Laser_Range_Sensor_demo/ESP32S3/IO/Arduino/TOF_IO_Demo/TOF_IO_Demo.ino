#define IO_H 16
#define IO_L 15

void setup(){
  Serial.begin(115200);
  pinMode(IO_H, INPUT); //Set IO_H to input mode 将IO_H设置成输入模式
  pinMode(IO_L, INPUT); //Set IO_L to input mode 将IO_L设置成输入模式
}

void loop(){
  if ((digitalRead(IO_H) == 1) && (digitalRead(IO_L) == 0)) //Read sensor status 读取传感器状态
    printf("In range.\r\n");
  else
    printf("Not in range.\r\n");
  delay(20);
}
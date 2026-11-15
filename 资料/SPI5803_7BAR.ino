
const int sclk = 10;
const int sen = 6;
const int sdi = 11;
const int sdo = 12;

/*
 ** 返回值 : temp--SPI读取的一字节数据
 ** 描  述 : 下降沿读数据，每次读取 1 bit
*/
uint8_t SPI_Read_OneByte(void)
{
  uint8_t i;
  uint8_t temp = 0;
  for (i = 0; i < 8; i++)
  {
    //读取MISO 8次输入的值，存入temp。之所以不放在“SCK = 0”语句之后的位置是因为：
    //读取最后1byte的最后一位(即LSB)之后，不能再左移了
    temp <<= 1;
    digitalWrite(sclk, HIGH);
    if (digitalRead(sdo) == HIGH) //读取最高位，保存至最末尾，通过左移位完成读整个字节
      temp |= 0x01;
    else
      temp &= ~0x01;
    digitalWrite(sclk, LOW);
  }
  return temp;
}

/*
** 参  数    : u8_writedata--SPI写入的一字节数据
** 描  述    : 上升沿写数据，每次写入 1 bit
*/
void SPI_Write_OneByte(uint8_t u8_writedata)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if (u8_writedata & 0x80) //判断最高位，总是发送最高位
    {
      digitalWrite(sdi, HIGH); //MOSI输出1，数据总线准备数据1
    }
    else
    {
      digitalWrite(sdi, LOW);//MOSI输出0，数据总线准备数据0
    }
    digitalWrite(sclk, HIGH); //上升沿来了(SCK从0-->1)，数据总线上的数据写入到器件
    u8_writedata <<= 1; //左移抛弃已经输出的最高位
    digitalWrite(sclk, LOW); //拉低SCK信号，初始化为0
  }
}
/*
** 返回值 : u8_readdata--SPI读取的一字节数据
** 参  数 : u8_writedata--SPI写入的一字节数据
** 描  述 : 上升沿写，下降沿读
*/
uint8_t SPI_WriteAndRead_OneByte(uint8_t u8_writedata)
{
  uint8_t i;
  uint8_t u8_readdata = 0x00;
  for (i = 0; i < 8; i++)
  {
    u8_readdata <<= 1;//读取MISO 8次输入的值，存入u8_readdata
    if (u8_writedata & 0x80)//判断最高位，总是写最高位（输出最高位）
      digitalWrite(sdi, HIGH);//MOSI输出1，数据总线准备数据1
    else
      digitalWrite(sdi, LOW);//MOSI输出0，数据总线准备数据0
    u8_writedata <<= 1;//左移抛弃已经输出的最高位
    digitalWrite(sclk, HIGH);//上升沿来了(SCK从0-->1)，数据总线上的数据写入器件
    if (digitalRead(sdo) == HIGH)//读取最高位，保存至最末尾，通过左移位完成读整个字节
      u8_readdata |= 0x01;
    else
      u8_readdata &= ~0x01;
    digitalWrite(sclk, LOW);//下降沿来了(SCK从1-->0)，MISO上将产生新的数据，读取存入u8——readdata
  }
  return u8_readdata;
}

uint8_t RawData[5];
uint8_t* GetSensorData()
{
  uint8_t buf[5];
  uint8_t error = 0;
  //重置清0
  memset(RawData, 0, sizeof(uint8_t) * 5);
  //采集温度数据
  wf5803_WriteReg(0x30, 0x08);
  delay(50);
  RawData[3] = wf5803_ReadReg(0x09);
  RawData[4] = wf5803_ReadReg(0x0A);
  
 /* wf5803_ReadRegMulti(0x0A,2,buf);
  RawData[3] = buf[1];
  RawData[4] = buf[0];*/
  //采集气压数据
  wf5803_WriteReg(0x30, 0x09);
  delay(50); 
  RawData[0] = wf5803_ReadReg(0x06);
  RawData[1] = wf5803_ReadReg(0x07);
  RawData[2] = wf5803_ReadReg(0x08);

  /* wf5803_ReadRegMulti(0x08,3,buf);
  RawData[0] = buf[2];
  RawData[1] = buf[1];
  RawData[2] = buf[0];*/
/*
  wf5803_ReadRegMulti(0x0A,5,buf);
  RawData[0] = buf[4];
  RawData[1] = buf[3];
  RawData[2] = buf[2];
  RawData[3] = buf[1];
  RawData[4] = buf[0];*/
  for(int i=0;i<5;i++){
    //Serial.print(RawData[i], HEX);
    //Serial.print(",");
  }
  return RawData;
}

/*
** 参 数 : (1)uint8_t  addr--寄存器地址
**         (2)uint8_t  value--写入值
** 说 明 : wf5803寄存器写函数
*/
void  wf5803_WriteReg(uint8_t addr, uint8_t value)
{
  digitalWrite(sen, LOW);//CS片选拉低
  SPI_Write_OneByte(0x00);//SPI写地址命令
  SPI_Write_OneByte(addr);
  SPI_Write_OneByte(value);//SPI写数据
  digitalWrite(sen, HIGH);//CS片选拉高
}

/*
** 返回值: value--读取寄存器值
** 参 数 : addr--寄存器地址
** 说 明 : wf5803寄存器读函数
*/
uint8_t  wf5803_ReadReg(uint8_t  addr)
{
  uint8_t value;
  digitalWrite(sen, LOW);//CS片选拉低
  SPI_Write_OneByte(0x80);//SPI写地址命令
  SPI_Write_OneByte(addr);
  value = SPI_Read_OneByte();//SPI读数据
  digitalWrite(sen, HIGH);//CS片选拉高
  return value;
}

void  wf5803_ReadRegMulti(uint8_t  addr, uint8_t lens, uint8_t* value)
{
  digitalWrite(sen, LOW);//CS片选拉低
  if (lens == 1)
  {
    SPI_Write_OneByte(0x80);//SPI写地址命令
  }
  else if (lens == 2)
  {
    SPI_Write_OneByte(0xA0);//SPI写地址命令
  }
  else if (lens == 3)
  {
    SPI_Write_OneByte(0xC0);//SPI写地址命令
  }
  else
  {
    SPI_Write_OneByte(0xE0);//SPI写地址命令
  }
  SPI_Write_OneByte(addr);
  for (uint8_t i = 0; i < lens; i++)
  {
    value[i] = SPI_Read_OneByte();//SPI读数据
    //  //Serial.print(i);
    //  //Serial.print(":0x");
    //  //Serial.println(value[i],HEX);
  }
  digitalWrite(sen, HIGH);//CS片选拉高
}

void idleStatus()
{
  digitalWrite(sen, HIGH);//CS片选拉高
  pinMode(sclk, INPUT);
 // pinMode(sen, INPUT);
 // pinMode(sdo, INPUT);
  pinMode(sdi, INPUT);
  delay(1);
}

void workStatus()
{
  pinMode(sclk, OUTPUT);
  pinMode(sen, OUTPUT);
  pinMode(sdo, INPUT);
  pinMode(sdi, OUTPUT);
  delay(1);
}

void setup() {
  workStatus();
  digitalWrite(sen, HIGH);//CS片选拉高

  Serial.begin(9600);

  //0x00:SDO_active: 1: 4-wire SPI, 0: 3-wire SPI
  wf5803_WriteReg(0x00, 0x01);
  delay(1);
  byte data = wf5803_ReadReg(0x00);
  //Serial.print("REG0:0x");
  //Serial.println(data, HEX);
  data = wf5803_ReadReg(0xA6);
  //Serial.print("REGA6:0x");
  //Serial.println(data, HEX);
  //000:1024X, 001:2048X, 010:4096X,011:8192X, 100:256X, 101:512X, 110:16384X, 111:32768X
  data = (data&0xF8) | (0b00000100);
  //Serial.println(data, HEX);
  wf5803_WriteReg(0xA6, data);
  data = wf5803_ReadReg(0xA6);
  //Serial.print("REGA6:0x");
  //Serial.println(data, HEX);
  //sleepMode
  wf5803_WriteReg(0x30, 0xFB);
  delay(1000);
}

long reading = 0;
float press = 0;
float temp = 0;
float fDat;
void loop() {
  
  uint8_t* p1 = GetSensorData();
  reading = p1[0];
  reading = reading << 8;
  reading |= p1[1];
  reading = reading << 8;
  reading |= p1[2];
  if (reading > 8388608) {
    fDat = (int32_t)(reading - 16777216) / 8388608.0f;
  } else {
    fDat = reading / 8388608.0f;
  }
  press = (1000 * (fDat - 0.1) + 50) * 10;//7bar
  Serial.print("press=");
  Serial.print(press);
  reading = p1[3];
  reading = reading << 8;
  reading |= p1[4];
  if (reading > 32768)
  {
    temp = (reading - 65536) / 256.0f;
  }
  else
  {
    temp = reading / 256.0f;
  }
  Serial.print(",temp=");
  Serial.println(temp);
  delay(500);
}

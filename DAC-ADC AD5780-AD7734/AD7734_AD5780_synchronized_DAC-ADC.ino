//Ardunio *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5780 DAC
//Andrea Young
//Carlos Kometter
#include "SPI.h" // necessary library for SPI communication
#include <vector>
int adc=52; //The SPI pin for the ADC
int dac[2]={4,5};  //The SPI pin for the DAC
int spi = 10;
int ldac=6; //Load DAC pin for DAC. Make it LOW if not in use. 
int reset=44 ; //Reset on ADC
int drdy=48; // Data is ready pin on ADC
int led = 32;
int data=28;//Used for trouble shooting; connect an LED between pin 13 and GND
int err=30;
const int Noperations = 13;
String operations[Noperations] = {"NOP", "INITIALIZE", "SET", "GET_DAC", "GET_ADC", "RAMP1", "RAMP2", "BUFFER_RAMP", "RESET", "TALK", "CONVERT_TIME", "*IDN?", "*RDY?"};
int initialized = 0;

namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(ldac,OUTPUT);   
  digitalWrite(ldac,HIGH); //Load DAC pin for DAC. Make it LOW if not in use. 
  pinMode(reset, OUTPUT);
  pinMode(drdy, INPUT);  //Data ready pin for the ADC.  
  pinMode(led, OUTPUT);  //Used for blinking indicator LED
  digitalWrite(led, HIGH);
  pinMode(data, OUTPUT);
  
  for(int i =0; i <= 1; i++)
  {
    pinMode(dac[i],OUTPUT);
    digitalWrite(dac[i],HIGH);
  }

  digitalWrite(reset,HIGH);  digitalWrite(data,LOW); digitalWrite(reset,LOW);  digitalWrite(data,HIGH); delay(5);  digitalWrite(reset,HIGH);  digitalWrite(data,LOW);//Resets ADC on startup.  

  SPI.begin(adc); // wake up the SPI bus for ADC
  SPI.begin(spi); // wake up the SPI bus for ADC
  
  SPI.setBitOrder(adc,MSBFIRST); //correct order for AD7734.
  SPI.setBitOrder(spi,MSBFIRST); //correct order for AD5764.
  SPI.setClockDivider(adc,84);  //This can probably be sped up now that the rest of the code is better optimized. Limited by ADC
  SPI.setClockDivider(spi,84);  //This can probably be sped up now that the rest of the code is better optimized. Limited by ADC
  SPI.setDataMode(adc,SPI_MODE3); //This should be 3 for the AD7734
  SPI.setDataMode(spi,SPI_MODE1); //This should be 1 for the AD5764

  // Disables DAC_SDO to avoid interference with ADC
//  SPI.transfer(dac,1,SPI_CONTINUE);
//  SPI.transfer(dac,0,SPI_CONTINUE);
//  SPI.transfer(dac,1);
}

void blinker(int s){digitalWrite(data,HIGH);delay(s);digitalWrite(data,LOW);delay(s);}
void sos(){blinker(50);blinker(50);blinker(50);blinker(500);blinker(500);blinker(500);blinker(50);blinker(50);blinker(50);}

void error()
{
  digitalWrite(err,HIGH);
  delay(3000);
  digitalWrite(err,LOW);
  delay(500);
}


int indexOfOperation(String operation)
{
  for(int index = 0; index < Noperations; index++)
  {
    if(operations[index] == operation)
    {
      return index;
    }
  }
  return 0;
}

void waitDRDY() {while (digitalRead(drdy)==HIGH){}}

void resetADC() //Resets the ADC, and sets the range to default +-10 V 
{
  digitalWrite(data,HIGH);digitalWrite(reset,HIGH);digitalWrite(reset,LOW);digitalWrite(reset,HIGH);
  SPI.transfer(adc,0x28);
  SPI.transfer(adc,0);
  SPI.transfer(adc,0x2A);
  SPI.transfer(adc,0);
}

void talkADC(std::vector<String> DB)
{
  int comm;
  comm=SPI.transfer(adc,DB[1].toInt());
  Serial.println(comm);
  Serial.flush();
}

int numberOfChannels(byte DB) // Returns the number of channels to write
{
  int number = 0;

  for(int i = 0; i <= 3; i++)
  {
    if(((DB >> i) & 1) == 1)
    {
       number++;
    }
  }
  return number;
}

std::vector<int> listOfChannels(byte DB) // Returns the list of channels to write
{
  std::vector<int> channels;

  int channel = 0;
  for(int i = 3; i >= 0; i--)
  {
    if(((DB >> i) & 1) == 1)
    {
       channels.push_back(channel);
    }
    channel++;
  }
  return channels;
}

void writeADCConversionTime(std::vector<String> DB)
{
  int adcChannel=DB[1].toInt();
  byte cr;

  byte fw = ((byte)(((DB[2].toFloat()*6.144-249)/128)+0.5))|128;

  SPI.transfer(adc,0x30+adcChannel);
  SPI.transfer(adc,fw);
  delayMicroseconds(100);
  SPI.transfer(adc,0x70+adcChannel);
  cr=SPI.transfer(adc,0); //Read back the CT register

  int convtime = ((int)(((cr&127)*128+249)/6.144)+0.5);
  Serial.println(convtime);
}

float map2(float x, long in_min, long in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int twoByteToInt(byte DB1,byte DB2) // This gives a 16 bit integer (between +/- 2^16)
{
  return ((int)((DB1<<8)| DB2));
}


void intToTwoByte(int s, byte * DB1, byte * DB2)
{
    *DB1 = ((byte)((s>>8)&0xFF));
    *DB2 = ((byte)(s&0xFF)); 
}


float twoByteToVoltage(byte DB1, byte DB2)
{
  int decimal;
  float voltage;

  decimal = twoByteToInt(DB1,DB2);

  if (decimal <= 32767)
  {
    voltage = decimal*10.0/32767;
  }
  else
  {
    voltage = -(65536-decimal)*10.0/32768;
  }
  return voltage;
}


void voltageToTwoByte(float voltage, byte * DB1, byte * DB2)
{
  int decimal;
  if (voltage > 10 || voltage < -10)
  {
    *DB1 = 128;
    *DB2 = 0;
    error();
  }  
  else if (voltage >= 0)
  {
    decimal = voltage*32767/10;
  }
  else
  {
    decimal = voltage*32768/10 + 65536;
  }
  intToTwoByte(decimal, DB1, DB2);
}

float getSingleReading(int adcchan)
{
  Serial.flush();
  int statusbyte=0;
  byte o2;
  byte o3;
  int ovr;
  if(adcchan <= 3)
  {
    SPI.transfer(adc,0x38+adcchan);   // Indicates comm register to access mode register with channel
    SPI.transfer(adc,0x48);           // Indicates mode register to start single convertion in dump mode
    waitDRDY();                       // Waits until convertion finishes
    SPI.transfer(adc,0x48+adcchan);   // Indcates comm register to read data channel data register
    statusbyte=SPI.transfer(adc,0);   // Reads Channel 'ch' status
    o2=SPI.transfer(adc,0);           // Reads first byte
    o3=SPI.transfer(adc,0);           // Reads second byte
    ovr=statusbyte&1;
    switch (ovr)
    {
      case 0:
      int decimal;
      decimal = twoByteToInt(o2,o3);
      float voltage;
      voltage = map2(decimal, 0, 65536, -10.0, 10.0);
      return voltage;
      break;
      
      case 1:
      return 0.0;
      break;   
    }
  }
}

float readADC(byte DB)
{
  int adcChannel=DB;
  switch (adcChannel)
  {
    case 0:
    return getSingleReading(0);
    break;
    case 1:
    return getSingleReading(2);
    break;
    case 2:
    return getSingleReading(1);
    break;
    case 3:
    return getSingleReading(3);
    break;

    default:  
    break;
  }
}

int threeByteToInt(byte DB1,byte DB2, byte DB3) // This gives a 16 bit integer (between +/- 2^16)
{
  return ((int)(((((DB1&15)<<8)| DB2)<<8)|DB3)>>2);
}

void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3)
{
  *DB1 = (byte)((decimal >> 14) | 16);
  *DB2 = (byte)((decimal >> 6) & 0xFF);
  *DB3 = (byte)((decimal & 0x3F) << 2);
}

float threeByteToVoltage(byte DB1, byte DB2, byte DB3)
{
  int decimal;
  float voltage;

  decimal = threeByteToInt(DB1,DB2,DB3);

  if (decimal <= 131071)
  {
    voltage = decimal*10.0/131071;
  }
  else
  {
    voltage = -(262144-decimal)*10.0/131072;
  }
  return voltage;
}

int voltageToDecimal(float voltage, byte *DB1, byte *DB2, byte *DB3)
{
  int decimal;
  if (voltage >= 0)
  {
    decimal = voltage*131071/10;
  }
  else
  {
    decimal = voltage*131072/10 + 262144;
  }
  intToThreeBytes(decimal, DB1, DB2, DB3);
}

float dacDataSend(int channelDAC, float voltage)
{
  byte b1;
  byte b2;
  byte b3;

  voltageToDecimal(voltage, &b1, &b2, &b3);
  SPI.transfer(spi,0);
  
  digitalWrite(data, HIGH);
  digitalWrite(channelDAC,LOW);
  SPI.transfer(spi,b1); // send command byte to DAC
  SPI.transfer(spi,b2); // MS data bits, DAC2
  SPI.transfer(spi,b3); //LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);

  digitalWrite(ldac,LOW);
  digitalWrite(ldac,HIGH);
  digitalWrite(data, LOW);

  return threeByteToVoltage(b1,b2,b3);
}

float writeDAC(int dacChannel, float voltage)
{
  switch(dacChannel)
  {
    case 0:
    return dacDataSend(dac[0],voltage);
    break;

    case 1:
    return dacDataSend(dac[1],voltage);
    break;

    default:
    break;
  }
}

void readingRampAvg(int adcchan, byte b1, byte b2, byte * o1, byte * o2,int count,int nReadings)
{
  Serial.flush();
  int statusbyte=0;
  int ovr;
  byte db1;
  byte db2;
  float sum=0;
  float avg;
  bool toSend = true;
  if(adcchan <= 3)
  {
    for(int i = 1; i<=nReadings; i++)
    {
      SPI.transfer(adc,0x38+adcchan);   // Indicates comm register to access mode register with channel
      SPI.transfer(adc,0x48);           // Indicates mode register to start single convertion in dump mode
      if (count>0 && toSend)
      {
        Serial.write(b1);                 // Sends previous reading while it is waiting for new reading
        Serial.write(b2);
        toSend = false;
      }
      waitDRDY();                       // Waits until convertion finishes
      SPI.transfer(adc,0x48+adcchan);   // Indcates comm register to read data channel data register
      statusbyte=SPI.transfer(adc,0);   // Reads Channel 'ch' status
      db1=SPI.transfer(adc,0);           // Reads first byte
      db2=SPI.transfer(adc,0);           // Reads second byte
      ovr=statusbyte&1;
      if (ovr){break;}
      int decimal = twoByteToInt(db1,db2);
      float voltage = map2(decimal, 0, 65536, -10.0, 10.0);
      sum += voltage;
    }
    if(ovr)
    {
      *o1=128;
      *o2=0;
    }
    else
    {
      avg = sum/nReadings;
      int decimal = map2(avg, -10.0, 10.0, 0, 65536);
      intToTwoByte(decimal, &db1, &db2);
      *o1=db1;
      *o2=db2;
    }
  }
}

void rampRead(byte DB,byte b1, byte b2, byte * o1, byte * o2, int count,int nReadings )
{
  int adcChannel=DB;
  switch (adcChannel)
  {
    case 0:
    readingRampAvg(0, b1 , b2, o1, o2,count,nReadings);
    break;
    
    case 1:
    readingRampAvg(2, b1 , b2, o1, o2,count,nReadings);
    break;
    
    case 2:
    readingRampAvg(1, b1 , b2, o1, o2,count,nReadings);
    break;
    
    case 3:
    readingRampAvg(3, b1 , b2, o1, o2,count,nReadings);
    break;
  
    default:  
    break;
  }
}

float dacDataSend_buffer(int channelDAC, float voltage)
{
  byte b1;
  byte b2;
  byte b3;

  voltageToDecimal(voltage, &b1, &b2, &b3);
  SPI.transfer(spi,0);
  
  digitalWrite(data, HIGH);
  digitalWrite(channelDAC,LOW);
  SPI.transfer(spi,b1); // send command byte to DAC
  SPI.transfer(spi,b2); // MS data bits, DAC2
  SPI.transfer(spi,b3); //LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);
  digitalWrite(data, LOW);

  return threeByteToVoltage(b1,b2,b3);
}

float writeDAC_buffer(int dacChannel, float voltage)
{
  switch(dacChannel)
  {
    case 0:
    return dacDataSend_buffer(dac[0],voltage);
    break;

    case 1:
    return dacDataSend_buffer(dac[1],voltage);
    break;

    default:
    break;
  }
}

void bufferRamp(std::vector<String> DB)
{
  String channelsDAC = DB[1];
  int NchannelsDAC = channelsDAC.length();
  String channelsADC = DB[2];
  int NchannelsADC = channelsADC.length();
  std::vector<float> vi;
  std::vector<float> vf;
  float v_min = -10.0;
  float v_max = 10.0;
  for(int i = 3; i < NchannelsDAC+3; i++)
  {
    vi.push_back(DB[i].toFloat());
    vf.push_back(DB[i+NchannelsDAC].toFloat());
  }
  int nSteps=(DB[NchannelsDAC*2+3].toInt());
  byte b1;
  byte b2;
  int count =0;
  for (int j=0; j<nSteps;j++)
  {
    digitalWrite(data,HIGH);
    for(int i = 0; i < NchannelsDAC; i++)
    {
      float v;
      v = vi[i]+(vf[i]-vi[i])*j/(nSteps-1);
      if(v<v_min)
      {
        v=v_min;
      }
      else if(v>v_max)
      {
        v=v_max;
      }
      writeDAC_buffer(channelsDAC[i]-'0',v);
    }
    digitalWrite(ldac,LOW);
    digitalWrite(ldac,HIGH);
    delayMicroseconds(DB[NchannelsDAC*2+4].toInt());
    for(int i = 0; i < NchannelsADC; i++)
    {
      rampRead(channelsADC[i]-'0', b1, b2, &b1, &b2, count,DB[NchannelsDAC*2+5].toInt());
      count+=1;
    }
  }
  Serial.write(b1);
  Serial.write(b2);
  digitalWrite(data,LOW);
}


void autoRamp1(std::vector<String> DB)
{
  float v1 = DB[2].toFloat();
  float v2 = DB[3].toFloat();
  int nSteps = DB[4].toInt();
  int dacChannel=DB[1].toInt();

  for (int j=0; j<nSteps;j++)
  {
    int timer = micros();
    digitalWrite(data,HIGH);
    writeDAC(dacChannel, v1+(v2-v1)*j/(nSteps-1));
    digitalWrite(data,LOW);
    while(micros() <= timer + DB[5].toInt());
  }
}

void autoRamp2(std::vector<String> DB)
{
  float vi1 = DB[3].toFloat();
  float vi2 = DB[4].toFloat();
  float vf1 = DB[5].toFloat();
  float vf2 = DB[6].toFloat();
  int nSteps = DB[7].toInt();
  byte b1;
  byte b2;
  int dacChannel1=DB[1].toInt();
  int dacChannel2=DB[2].toInt();

  for (int j=0; j<nSteps;j++)
  {
    int timer = micros();
    digitalWrite(data,HIGH);
    writeDAC(dacChannel1, vi1+(vf1-vi1)*j/(nSteps-1));
    writeDAC(dacChannel2, vi2+(vf2-vi2)*j/(nSteps-1));
    while(micros() <= timer + DB[8].toInt());
    digitalWrite(data,LOW);
  }
}

//void autoRamp(std::vector<byte> DB)
//{
//  int v1=twoByteToVoltage(DB[1],DB[2]);
//  int v2=twoByteToVoltage(DB[3],DB[4]);
//  int nSteps=(DB[5]);
//  byte b1;
//  byte b2;
//  int dacChannel=(DB[0])&7;
//
//  for (int j=0; j<nSteps;j++)
//  {
//    digitalWrite(led,LOW);
//    voltageToTwoByte(v1+(v2-v1)*j/(nSteps-1), &b1, &b2);
//    dacDataSend(dacChannel,b1,b2);
//    delayMicroseconds(50);
//    readADC(DB[0]);
//    digitalWrite(led,HIGH);
//  }
//  digitalWrite(led,LOW);
//}


void readDAC(int channelDAC)
{
  
  int o1;
  int o2;
  int o3;
  float voltage;
  
  digitalWrite(channelDAC,LOW);
  SPI.transfer(spi,144); // send command byte to DAC
  SPI.transfer(spi,0); // MS data bits, DAC2
  SPI.transfer(spi,0);//LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);
  delayMicroseconds(1);
  digitalWrite(channelDAC,LOW);
  o1 = SPI.transfer(spi,0);
  o2 = SPI.transfer(spi,0);
  o3 = SPI.transfer(spi,0);
  digitalWrite(channelDAC,HIGH);
  voltage = threeByteToVoltage(o1, o2, o3);
  Serial.println(voltage,5);
}

void normalMode()
{
  digitalWrite(data,HIGH);
  int attemps = 0;
  for( int i = 0; i <= 1; i++)
  {
    int o;
    for(int j=0; j<=3; j++)
    {
      dacDataSend(dac[i],0);
      digitalWrite(dac[i],LOW);
      SPI.transfer(spi,32);
      SPI.transfer(spi,0);
      SPI.transfer(spi,2);
      digitalWrite(dac[i],HIGH);
  
      digitalWrite(dac[i],LOW);
      SPI.transfer(spi,160);
      SPI.transfer(spi,0);
      SPI.transfer(spi,0);
      digitalWrite(dac[i],HIGH);
      delayMicroseconds(1);
      digitalWrite(dac[i],LOW);
      SPI.transfer(spi,0);
      SPI.transfer(spi,0);
      o = SPI.transfer(spi,0);
      digitalWrite(dac[i],HIGH);
    }
    
    if (attemps>=5)
    {
      Serial.print("ERROR INITIALIZING DAC");
      Serial.println(i);
    }
    else if (o!=2)
    {
      i=i-1;
      attemps++;
    };
  }
  digitalWrite(data,LOW);
}

void ID()
{
  Serial.println("DAC-ADC_AD7734-AD5780");
}

void RDY()
{
  Serial.println("READY");
}

void debug()
{
  digitalWrite(data,HIGH);
  delay(3000);
  digitalWrite(data,LOW);
  delay(3000);
}


void router(std::vector<String> DB)
{
  float v;
  int operation = indexOfOperation(DB[0]);
  switch ( operation )
  {
    case 0:
    Serial.println("NOP");
    break;
    
    case 1:
//    debug();
    normalMode();
    Serial.println("INITIALIZATION COMPLETE");
    break;

    case 2: // Write DAC **IMPLEMENTED, WORKS
    if(DB[2].toFloat() < -10 || DB[2].toFloat() > 10)
    {
      Serial.println("VOLTAGE_OVERRANGE");
      break;
    }
    v = writeDAC(DB[1].toInt(),DB[2].toFloat());
    Serial.print("DAC ");
    Serial.print(DB[1]);
    Serial.print(" UPDATED TO ");
    Serial.print(v,5);
    Serial.println("V");
    break;
    
    case 3:
    int channel;
    channel = dac[DB[1].toInt()];
    readDAC(channel);
    break;
    
    case 4: // Read ADC 
    v=readADC(DB[1].toInt());
    Serial.println(v,4);
    break;

    case 5:
    autoRamp1(DB);
    Serial.println("RAMP_FINISHED");
    break;

    case 6:
    autoRamp2(DB);
    Serial.println("RAMP_FINISHED");
    break;
    
    case 7: // Autoramp
    bufferRamp(DB);
    Serial.println("BUFFER_RAMP_FINISHED");
    break;

    case 8:
    resetADC();
    break;
    
    case 9:
    talkADC(DB);
    break;
    
    case 10: // Write conversion time registers
    writeADCConversionTime(DB);
    break;

    case 11: // ID
    ID();
    break;

    case 12:
    RDY();
    break;

    default:
    break;
  }
}

void loop() 
{
  Serial.flush();
  String inByte = "";
  std::vector<String> comm;
  
  if(initialized == 0)
  {
    normalMode();
    initialized = 1;
  }
  
  if(Serial.available())
  {
    char received;
    while (received != '\r')
    {
      if(Serial.available())
      {
        received = Serial.read();
        if (received == '\n' || received == ' ')
        {}
        else if (received == ',' || received == '\r')
        {
          comm.push_back(inByte);
          inByte = "";
        }
        else
        {
          inByte += received;
        }        
      }
    }
  router(comm);
  }
}

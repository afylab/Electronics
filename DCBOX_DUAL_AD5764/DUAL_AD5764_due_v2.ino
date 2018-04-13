//Ardunio *DUE* code for controlling two EVAL-AD5764 DACs
//Created by Andrea Young
//Modified by Carlos Kometter 7/24/2015
#include <vector>
#include "SPI.h" // necessary library for SPI communication
int cs=52;  //The SPI pin for the DAC
int ldac=6; //Load DAC pin for DAC. Make it LOW if not in use. 
int clr=7;  // Asynchronous clear pin for DAC. Make it HIGH if you are not using it
int led=46;
int data=48;
const int Noperations = 7;
String operations[Noperations] = {"NOP", "SET", "GET_DAC", "RAMP1", "RAMP2", "*IDN?", "*RDY?"};

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
  while (! Serial);
  pinMode(led,OUTPUT);   
  pinMode(data,OUTPUT);   
  pinMode(ldac,OUTPUT);   
  digitalWrite(ldac,HIGH); //Load DAC pin for DAC. Make it LOW if not in use. 
  pinMode(clr, OUTPUT);
  digitalWrite(clr,HIGH); // Asynchronous clear pin for DAC. Make it HIGH if you are not using it

  SPI.begin(cs); // wake up the SPI bus for DAC1
  
  SPI.setBitOrder(cs,MSBFIRST); //correct order for AD7732.
  SPI.setClockDivider(cs,84);  //This can probably be sped up now that the rest of the code is better optimized. Limited by ADC
  SPI.setDataMode(cs,SPI_MODE1); //This should be 3 for the AD7732
  digitalWrite(led,HIGH);
  digitalWrite(data,LOW);
}

void blinker(int s){digitalWrite(data,HIGH);delay(s);digitalWrite(data,LOW);delay(s);}

void sos(int s){blinker(s);blinker(s);blinker(s);blinker(s);blinker(s);}

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

int twoByteToInt(byte DB1,byte DB2) // This gives a 16 bit integer (between +/- 2^16)
{
  return ((int)((DB1<<8)| DB2));
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

void intToTwoByte(int s, byte * DB1, byte * DB2)
{
    *DB1 = ((byte)((s>>8)&0xFF));
    *DB2 = ((byte)(s&0xFF)); 
}

void voltageToTwoByte(float voltage, byte * DB1, byte * DB2)
{
  int decimal;
  if (voltage > 10 || voltage < -10)
  {
    *DB1 = 0;
    *DB2 = 0;
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


float setValue(int dacChannel1, int dacChannel2, float v1, float v2)
{
  byte b1;
  byte b2;
  byte b3;
  byte b4;
  digitalWrite(data, HIGH);
  voltageToTwoByte(v1,&b1,&b2);
  voltageToTwoByte(v2,&b3,&b4);
  SPI.transfer(cs,16+dacChannel1,SPI_CONTINUE); // send command byte to DAC2 in the daisy chain.
  SPI.transfer(cs,b1,SPI_CONTINUE); // MS data bits, DAC2
  SPI.transfer(cs,b2,SPI_CONTINUE);//LS 8 data bits, DAC2
  SPI.transfer(cs,16+dacChannel2,SPI_CONTINUE);// send command byte to DAC1 in the daisy chain.
  SPI.transfer(cs,b3,SPI_CONTINUE);// MS data bits, DAC1
  SPI.transfer(cs,b4);//LS 8 data bits, DAC1
  digitalWrite(ldac,LOW);
  delayMicroseconds(0.003);
  digitalWrite(ldac,HIGH);
  digitalWrite(data, LOW);

  if (dacChannel1 == -16)
  {
   return twoByteToVoltage(b3, b4); 
  }
  else
  {
    return twoByteToVoltage(b1, b2);
  }
}

float writeDAC(int dacChannel, float voltage)
{
  switch(dacChannel)
  {
    case 0:
    return setValue(2, -16, voltage, 0.0);
    break;

    case 1:
    return setValue(0, -16, voltage, 0.0);
    break;

    case 2:
    return setValue(-16, 2, 0.0, voltage);
    break;

    case 3:
    return setValue(-16, 0, 0.0, voltage);
    break;

    case 4:
    return setValue(3, -16, voltage, 0.0);
    break;

    case 5:
    return setValue(1, -16, voltage, 0.0);
    break;

    case 6:
    return setValue(-16, 3, 0.0, voltage);
    break;

    case 7:
    return setValue(-16, 1, 0.0, voltage);
    break;

    default:
    break;
  }
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

void getValue(int dacChannel, int dac)
{
  SPI.transfer(cs,144+dacChannel,SPI_CONTINUE); // send command byte to DAC2 in the daisy chain.
  SPI.transfer(cs,0,SPI_CONTINUE); // MS data bits, DAC2
  SPI.transfer(cs,0,SPI_CONTINUE);//LS 8 data bits, DAC2
  SPI.transfer(cs,144+dacChannel,SPI_CONTINUE);// send command byte to DAC1 in the daisy chain.
  SPI.transfer(cs,0,SPI_CONTINUE);// MS data bits, DAC1
  SPI.transfer(cs,0);//LS 8 data bits, DAC1

  SPI.transfer(cs,0,SPI_CONTINUE); // send command byte to DAC2 in the daisy chain.
  byte o2 = SPI.transfer(cs,0,SPI_CONTINUE); // MS data bits, DAC2
  byte o3 = SPI.transfer(cs,0,SPI_CONTINUE);//LS 8 data bits, DAC2
  SPI.transfer(cs,0,SPI_CONTINUE);// send command byte to DAC1 in the daisy chain.
  byte o5 = SPI.transfer(cs,0,SPI_CONTINUE);// MS data bits, DAC1
  byte o6 = SPI.transfer(cs,0);//LS 8 data bits, DAC1

  if(dac)
  {
    float voltage = twoByteToVoltage(o5, o6);
    Serial.println(voltage,4);
  }
  else
  {
    float voltage = twoByteToVoltage(o2, o3);
    Serial.println(voltage,4);
  }
}

void readDAC(int dacChannel)
{
  switch(dacChannel)
  {
    case 0:
    getValue(2, 0);
    break;

    case 1:
    getValue(0, 0);
    break;

    case 2:
    getValue(2, 1);
    break;

    case 3:
    getValue(0, 1);
    break;

    case 4:
    getValue(3, 0);
    break;

    case 5:
    getValue(1, 0);
    break;

    case 6:
    getValue(3, 1);
    break;

    case 7:
    getValue(1, 1);
    break;

    default:
    break;
  }
}

void ID()
{
  Serial.println("DCBOX_DUAL_AD5764");
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
  int operation = indexOfOperation(DB[0]);
  switch( operation )//uses the first byte choose what to do.  0x00: set DACS with asynchronous update
  {
    case 0:
    Serial.println("NOP");
    break;
    
    case 1: // Write DAC;
    if(DB[2].toFloat() < -10 || DB[2].toFloat() > 10)
    {
      Serial.println("VOLTAGE_OVERRANGE");
      break;
    }
    float v; 
    v = writeDAC(DB[1].toInt(),DB[2].toFloat());
    Serial.print("DAC ");
    Serial.print(DB[1]);
    Serial.print(" UPDATED TO ");
    Serial.print(v,4);
    Serial.println("V");
    break;

    case 2:
    readDAC(DB[1].toInt());
    break;

    case 3:
    autoRamp1(DB);
    Serial.println("RAMP_FINISHED");
    break;

    case 4:
    autoRamp2(DB);
    Serial.println("RAMP_FINISHED");
    break;

    case 5: // ID
    ID();
    break;

    case 6:
    RDY();
    break;

    default:
    sos(20);
    break;
  }
}
void loop() 
{
  Serial.flush();
  String inByte = "";
  std::vector<String> comm;
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

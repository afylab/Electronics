//Ardunio *DUE*code for controlling EVAL-AD5780
//Created by Carlos Kometter

/////////////CHANGELOG//////////////////
//v3- 5/20/2016  If a DAC does not intilizies, normalmode() is recall until everysingle dac is initilized. 
//v4-  6/15/2016  Initializes dacs durin power up
/////////////////////////////////

#include <vector>
#include "SPI.h" // necessary library for SPI communication
int ldac = 11;
int led = 30;
int data = 32;
int dac[4] = {4, 10, 12, 52};
const int Noperations = 8;
String operations[Noperations] = {"NOP", "INITIALIZE", "SET", "GET_DAC", "RAMP1", "RAMP2", "*IDN?", "RDY?"}; //NOP, *IDN?, *RDY? RAMP
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
  while (! Serial);
  pinMode(ldac,OUTPUT);   
  digitalWrite(ldac,LOW); //Load DAC pin for DAC. Make it LOW if not in use. 
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);

  for(int i =0; i <= 3; i++)
  {
    pinMode(dac[i],OUTPUT);
    digitalWrite(dac[i],HIGH);
  }
  

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(84);
  SPI.setDataMode(SPI_MODE1);   
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

float setValue(int channelDAC, float voltage)
{
  byte b1;
  byte b2;
  byte b3;

  voltageToDecimal(voltage, &b1, &b2, &b3);

  digitalWrite(data, HIGH);
  digitalWrite(channelDAC,LOW);
  SPI.transfer(b1); // send command byte to DAC
  SPI.transfer(b2); // MS data bits, DAC2
  SPI.transfer(b3); //LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);
  digitalWrite(data, LOW);

  return threeByteToVoltage(b1,b2,b3);
}

float writeDAC(int dacChannel, float voltage)
{
  switch( dacChannel )//uses the first byte choose what to do.  0x00: set DACS with asynchronous update
  {
    case 0: // Write DAC;
    return setValue(dac[0],voltage);
    break;

    case 1:
    return setValue(dac[1],voltage);
    break;

    case 2:
    return setValue(dac[2],voltage);
    break;

    case 3:
    return setValue(dac[3],voltage);
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
    writeDAC(dacChannel, v1+(v2-v1)*j/(nSteps-1));
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
    writeDAC(dacChannel1, vi1+(vf1-vi1)*j/(nSteps-1));
    writeDAC(dacChannel2, vi2+(vf2-vi2)*j/(nSteps-1));
    while(micros() <= timer + DB[8].toInt());
  }
}

void readControl(int channelDAC)
{
  int o1;
  int o2;
  int o3;

  digitalWrite(channelDAC,LOW);
  SPI.transfer(160); // send command byte to DAC
  SPI.transfer(0); // MS data bits, DAC2
  SPI.transfer(0);//LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);
  delayMicroseconds(3);
  digitalWrite(channelDAC,LOW);
  o1 = SPI.transfer(0);
  o2 = SPI.transfer(0);
  o3 = SPI.transfer(0);
  digitalWrite(channelDAC,HIGH);

  Serial.write(o1);
  Serial.write(o1);
  Serial.write(o1);
}

void readDAC(int channelDAC)
{
  int o1;
  int o2;
  int o3;
  float voltage;
  
  digitalWrite(channelDAC,LOW);
  SPI.transfer(144); // send command byte to DAC
  SPI.transfer(0); // MS data bits, DAC2
  SPI.transfer(0);//LS 8 data bits, DAC2
  digitalWrite(channelDAC,HIGH);
  delayMicroseconds(1);
  digitalWrite(channelDAC,LOW);
  o1 = SPI.transfer(0);
  o2 = SPI.transfer(0);
  o3 = SPI.transfer(0);
  digitalWrite(channelDAC,HIGH);
  
  voltage = threeByteToVoltage(o1, o2, o3);
  Serial.println(voltage,4);
}

void normalMode()
{
  int attemps = 0;
  for( int i = 0; i <= 3; i++)
  {
    int o;
    setValue(dac[i],0);
    digitalWrite(dac[i],LOW);
    SPI.transfer(32);
    SPI.transfer(0);
    SPI.transfer(2);
    digitalWrite(dac[i],HIGH);

    digitalWrite(dac[i],LOW);
    SPI.transfer(160);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(dac[i],HIGH);
    delayMicroseconds(1);
    digitalWrite(dac[i],LOW);
    SPI.transfer(0);
    SPI.transfer(0);
    o = SPI.transfer(0);
    digitalWrite(dac[i],HIGH);

    if (attemps>=5)
    {
      Serial.print("ERROR INITIALIZING DAC");
      Serial.println(i);
    }
    else if (o!=2)
    {
      i=i-1;
      attemps++;
    }
  }
}

void ID()
{
  Serial.println("DCBOX_QUAD_AD5780(DOGWOOD)");
}

void RDY()
{
  Serial.println("READY");
}

void debug()
{
  digitalWrite(led,HIGH);
  delay(3000);
  digitalWrite(led,LOW);
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
    normalMode();
    Serial.println("INITIALIZATION COMPLETE");
    break;

    case 2:
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
    Serial.print(v,5);
    Serial.println("V");
    break;

    case 3:
    readDAC(dac[DB[1].toInt()]);
    break;

    case 4:
    autoRamp1(DB);
    Serial.println("RAMP_FINISHED");
    break;

    case 5:
    autoRamp2(DB);
    Serial.println("RAMP_FINISHED");
    break;

    case 6:
    ID();
    break;

    case 7:
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

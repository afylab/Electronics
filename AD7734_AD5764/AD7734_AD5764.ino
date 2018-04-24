//Ardunio *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5764 DAC
//Andrea Young
//Carlos Kometter
#include "SPI.h" // necessary library for SPI communication
#include <vector>
int adc=52; //The SPI pin for the ADC
int spi = 4; //The SPI pin for the DAC
int ldac=6; //Load DAC pin for DAC. Make it LOW if not in use. 
int reset=44 ; //Reset on ADC
int drdy=48; // Data is ready pin on ADC
int led = 32;
int data=28;//Used for trouble shooting; connect an LED between pin 28 and GND
int err=30;
const int Noperations = 21;
String operations[Noperations] = {"NOP", "INITIALIZE", "SET", "GET_DAC", "GET_ADC", "RAMP1", "RAMP2", "BUFFER_RAMP", "BUFFER_RAMP_DIS", "RESET", "TALK", "CONVERT_TIME", "*IDN?", "*RDY?", "GET_DUNIT","SET_DUNIT", "ADC_ZERO_SC_CAL", "ADC_CH_ZERO_SC_CAL", "ADC_CH_FULL_SC_CAL", "DAC_CH_CAL", "FULL_SCALE"};
int delayUnit=0; // 0=microseconds 1=miliseconds

float DAC_FULL_SCALE = 10;

// Calibration constans
float OS[4]={0,0,0,0}; // offset error
float GE[4]={1,1,1,1}; // gain error

std::vector<String> query_serial()
{
  char received;
  String inByte = "";
  std::vector<String> comm;
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
  return comm;
}

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
  
  digitalWrite(reset,HIGH);  digitalWrite(data,LOW); digitalWrite(reset,LOW);  digitalWrite(data,HIGH); delay(5);  digitalWrite(reset,HIGH);  digitalWrite(data,LOW);//Resets ADC on startup.  

  SPI.begin(adc); // wake up the SPI bus for ADC
  SPI.begin(spi); // wake up the SPI bus for ADC
  
  SPI.setBitOrder(adc,MSBFIRST); //correct order for AD7734.
  SPI.setBitOrder(spi,MSBFIRST); //correct order for AD5764.
  SPI.setClockDivider(adc,7); // Maximum 12 Mhz for AD7734
  SPI.setClockDivider(spi,4); // Maximum 21 Mhz for AD5764
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

void writeADCConversionTime(std::vector<String> DB)
{
  int adcChannel;
  switch(DB[1].toInt()){
    case 0:
    adcChannel = 0;
    break;
    case 1:
    adcChannel = 1;
    break;
    case 2:
    adcChannel = 2;
    break;
    case 3:
    adcChannel = 3;
    break;

    default:  
    break;
  }
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

float map2(float x, long in_min, long in_max, float out_min, float out_max) //float
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
  if (voltage > DAC_FULL_SCALE || voltage < -1*DAC_FULL_SCALE)
  {
    *DB1 = 128;
    *DB2 = 0;
    error();
  }  
  else if (voltage >= 0)
  {
    decimal = voltage*32767/DAC_FULL_SCALE;
  }
  else
  {
    decimal = voltage*32768/DAC_FULL_SCALE + 65536;
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
    return getSingleReading(1);
    break;
    case 2:
    return getSingleReading(2);
    break;
    case 3:
    return getSingleReading(3);
    break;

    default:  
    break;
  }
}

float dacDataSend(int ch, float voltage)
{
  byte b1;
  byte b2;

  voltageToTwoByte(voltage, &b1, &b2);
  
  digitalWrite(data, HIGH);
  SPI.transfer(spi,16+ch,SPI_CONTINUE); // Indicates to DAC to write channel 'ch' in the data register 
  SPI.transfer(spi,b1,SPI_CONTINUE);   // writes first byte
  SPI.transfer(spi,b2);                // writes second byte

  digitalWrite(ldac,LOW);
  
  digitalWrite(ldac,HIGH);
  digitalWrite(data, LOW);

  return twoByteToVoltage(b1, b2);
}

float writeDAC(int dacChannel, float voltage)
{
  switch(dacChannel)
  {
    case 0:
    return dacDataSend(dacChannel,voltage/GE[0]-OS[0]);
    break;

    case 1:
    return dacDataSend(dacChannel,voltage/GE[1]-OS[1]);
    break;

    case 2:
    return dacDataSend(dacChannel,voltage/GE[2]-OS[2]);
    break;

    case 3:
    return dacDataSend(dacChannel,voltage/GE[3]-OS[3]);
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
    readingRampAvg(1, b1 , b2, o1, o2,count,nReadings);
    break;
    
    case 2:
    readingRampAvg(2, b1 , b2, o1, o2,count,nReadings);
    break;
    
    case 3:
    readingRampAvg(3, b1 , b2, o1, o2,count,nReadings);
    break;
  
    default:  
    break;
  }
}

float dacDataSend_buffer(int ch, float voltage)
{
  byte b1;
  byte b2;

  voltageToTwoByte(voltage, &b1, &b2);
  
  digitalWrite(data, HIGH);
  SPI.transfer(spi,16+ch,SPI_CONTINUE); // Indicates to DAC to write channel 'ch' in the data register 
  SPI.transfer(spi,b1,SPI_CONTINUE);   // writes first byte
  SPI.transfer(spi,b2);                // writes second byte
  digitalWrite(data, LOW);

  return twoByteToVoltage(b1, b2);
}

float writeDAC_buffer(int dacChannel, float voltage)
{
  switch(dacChannel)
  {
    case 0:
    return dacDataSend_buffer(dacChannel,voltage/GE[0]-OS[0]);
    break;

    case 1:
    return dacDataSend_buffer(dacChannel,voltage/GE[1]-OS[1]);
    break;

    case 2:
    return dacDataSend_buffer(dacChannel,voltage/GE[2]-OS[2]);
    break;

    case 3:
    return dacDataSend_buffer(dacChannel,voltage/GE[3]-OS[3]);
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
  float v_min = -1*DAC_FULL_SCALE;
  float v_max = DAC_FULL_SCALE;
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
    if (delayUnit)
    {
      delay(DB[NchannelsDAC*2+4].toInt());
    }
    else
    {
      delayMicroseconds(DB[NchannelsDAC*2+4].toInt());
    }
    for(int i = 0; i < NchannelsADC; i++)
    {
      rampRead(channelsADC[i]-'0', b1, b2, &b1, &b2, count,DB[NchannelsDAC*2+5].toInt());
      count+=1;
    }
    if(Serial.available())
    {
      std::vector<String> comm;
      comm = query_serial();
      if(comm[0] == "STOP")
      {
        break;
      }
    }
  }
  Serial.write(b1);
  Serial.write(b2);
  digitalWrite(data,LOW);
}

int bufferRampDis(std::vector<String> DB)
{
  String channelsDAC = DB[1];
  int NchannelsDAC = channelsDAC.length();
  String channelsADC = DB[2];
  int NchannelsADC = channelsADC.length();
  int nAdcSteps = DB[NchannelsDAC*2+6].toInt();
  int nSteps=(DB[NchannelsDAC*2+3].toInt());

  if (nAdcSteps > nSteps)
  {
    Serial.println("nAdcSteps must be larger or equal to nSteps");
    return 0;
  }

  std::vector<float> vi;
  std::vector<float> vf;
  float v_min = -1*DAC_FULL_SCALE;
  float v_max = DAC_FULL_SCALE;
  for(int i = 3; i < NchannelsDAC+3; i++)
  {
    vi.push_back(DB[i].toFloat());
    vf.push_back(DB[i+NchannelsDAC].toFloat());
  }
  byte b1;
  byte b2;
  int count =0;
  int adcCount = 0;
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
    if (delayUnit)
    {
      delay(DB[NchannelsDAC*2+4].toInt());
    }
    else
    {
      delayMicroseconds(DB[NchannelsDAC*2+4].toInt());
    }
    if (j == int(adcCount*float(nSteps-1)/float(nAdcSteps-1)))
    {
      for(int i = 0; i < NchannelsADC; i++)
      {
        rampRead(channelsADC[i]-'0', b1, b2, &b1, &b2, count,DB[NchannelsDAC*2+5].toInt());
        count+=1;
      }
      adcCount+=1;
    }
    if(Serial.available())
    {
      std::vector<String> comm;
      comm = query_serial();
      if(comm[0] == "STOP")
      {
        break;
      }
    }
  }
  Serial.write(b1);
  Serial.write(b2);
  digitalWrite(data,LOW);
  return 0;
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

void ID()
{
  Serial.println("DAC-ADC_AD7734-AD5764");
}

void RDY()
{
  Serial.println("READY");
}

void setUnit(int unit)
{
  if (unit == 0)
  {
    delayUnit = 0;
    Serial.println("Delay unit set to microseconds");
  }
  else if (unit == 1)
  {
    delayUnit = 1;
    Serial.println("Delay unit set to miliseconds");
  }
  else
  {
    Serial.println("Unit should be 0 (microseconds) or 1 (miliseconds)");
  }
}

void adc_zero_scale_cal()
{
  SPI.transfer(adc,0x38);   // Indicates comm register to access mode register
  SPI.transfer(adc,0x80);
  waitDRDY();
}

void adc_ch_zero_scale_cal()
{
  SPI.transfer(adc,0x38);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xC0);
  waitDRDY();

  SPI.transfer(adc,0x38+1);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xC0);
  waitDRDY();

  SPI.transfer(adc,0x38+2);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xC0);
  waitDRDY();

  SPI.transfer(adc,0x38+3);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xC0);
  waitDRDY();
}

void adc_ch_full_scale_cal()
{
  SPI.transfer(adc,0x38);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xE0);
  waitDRDY();

  SPI.transfer(adc,0x38+1);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xE0);
  waitDRDY();

  SPI.transfer(adc,0x38+2);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xE0);
  waitDRDY();

  SPI.transfer(adc,0x38+3);   // Indicates comm register to access mode register
  SPI.transfer(adc,0xE0);
  waitDRDY();
}

void dac_ch_cal()
{
  for(int i=0; i<=3; i++)
  {
    OS[i]=0; // offset error
    GE[i]=1; // gain error
  }
  //set dacs to zero volts
  for(int i=0; i<=3; i++)
  {
    dacDataSend(i,0);
  }
  delay(1);
  //reads the offset of each channel
  for(int i=0; i<=3; i++)
  {
    OS[i]=readADC(i);
  }
  
  //set dacs to 9 volts
  float ifs = 9;
  for(int i=0; i<=3; i++)
  {
    dacDataSend(i,ifs);
  }
  delay(1);
  //reads each channel and calculates the gain error
  for(int i=0; i<=3; i++)
  {
    float nfs;
    nfs=readADC(i);
    GE[i]=(nfs-OS[i])/ifs;
  }
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
    Serial.println("NOP");
    break;

    case 2:
    if(DB[2].toFloat() < -1*DAC_FULL_SCALE || DB[2].toFloat() > DAC_FULL_SCALE)
    {
      Serial.println("VOLTAGE_OVERRANGE");
      break;
    }
    v = writeDAC(DB[1].toInt(),DB[2].toFloat());
    Serial.print("DAC ");
    Serial.print(DB[1]);
    Serial.print(" UPDATED TO ");
    Serial.print(v,4);
    Serial.println("V");
    break;
    
    case 3:
    Serial.println("GET DAC NOT IMPLEMENTED");
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
    bufferRampDis(DB);
    Serial.println("BUFFER_RAMP_FINISHED");
    break;

    case 9:
    resetADC();
    break;
    
    case 10:
    talkADC(DB);
    break;
    
    case 11: // Write conversion time registers
    writeADCConversionTime(DB);
    break;

    case 12: // ID
    ID();
    break;

    case 13:
    RDY();
    break;

    case 14:
    Serial.println(delayUnit);
    break;

    case 15:
    setUnit(DB[1].toInt()); // 0 = microseconds 1 = miliseconds
    break;

    case 16:
    adc_zero_scale_cal();
    Serial.println("CALIBRATION_FINISHED");
    break;

    case 17:
    adc_ch_zero_scale_cal();
    Serial.println("CALIBRATION_FINISHED");
    break;

    case 18:
    adc_ch_full_scale_cal();
    Serial.println("CALIBRATION_FINISHED");
    break;

    case 19:
    dac_ch_cal();
    Serial.println("CALIBRATION_FINISHED");

    case 20:
    DAC_FULL_SCALE = DB[1].toFloat();
    Serial.println("FULL_SCALE_UPDATED");
    break;
    
    default:
    break;
  }
}

void loop() 
{
  Serial.flush();
  std::vector<String> comm;
  
  if(Serial.available())
  {
    comm = query_serial();
    router(comm);
  }
}

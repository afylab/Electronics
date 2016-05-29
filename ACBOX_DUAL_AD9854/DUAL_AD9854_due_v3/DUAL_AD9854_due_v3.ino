//Arduino DUE program for AC box
//Created by Brunel Odegard
//Based on code by Andrea Young and Carlos Kometter
#include <vector>
#include "SPI.h"

int update  = 46;
int mreset  =  9;
int ioreset = 34;
int cs1     =  4;
int cs2     = 52;
int led1    = 49;
int led2    = 51;
int led3    = 53;

int refclock  = 20; // in MHz, hence the pow(10,6) below
int clockmult =  5;
int sysclock  = clockmult*refclock*(pow(10,6));

void setup()
{
  Serial.begin(115200);
  pinMode(led1   ,OUTPUT);
  pinMode(led2   ,OUTPUT);
  pinMode(led3   ,OUTPUT);
  pinMode(43     ,OUTPUT);
  pinMode(2      ,OUTPUT);
  pinMode(mreset ,OUTPUT);
  pinMode(ioreset,OUTPUT);
  pinMode(update ,OUTPUT);
  digitalWrite(mreset ,LOW);
  digitalWrite(ioreset,LOW);
  digitalWrite(update ,LOW);
  digitalWrite(led1   ,LOW);
  digitalWrite(led2   ,LOW);
  digitalWrite(led3   ,LOW);
  SPI.begin(cs1);
  SPI.begin(cs2);
  SPI.setBitOrder(cs1,MSBFIRST);
  SPI.setBitOrder(cs2,MSBFIRST);
  SPI.setClockDivider(cs1,84);
  SPI.setClockDivider(cs2,84);
  SPI.setDataMode(cs1,SPI_MODE0); // Rising edge mode
  SPI.setDataMode(cs2,SPI_MODE0);
  for (int i=0;i<3;i++)
  {
    flash(led1,100); // Initialization sequence
    flash(led2,100); // Flashes each LED
    flash(led3,100); // three times in order.
  }
}

void flash(int port, int ms)
{
  digitalWrite(port,HIGH);
  delay(ms);
  digitalWrite(port,LOW);
}

// remembering internal settings for GET and ? commands
float  channels_[4] = {0.0,0.0,0.0,0.0};
float  phase_[1]    = {0.0}            ;
double frequency_   = 0.0              ;


const int nOperations = 12;
String operations[nOperations] = {"NOP","INIT","SET","PHS","FRQ","*IDN?","*RDY?","MR","UPD","GET","PHS?","FRQ?"};
int indexOfOperation(String op)
{
  for (int i=0; i<nOperations; i++)
  {
    if (operations[i] == op)
    {
      return i;
    }
  }
  return 0;
}

void control_register(int chip, int clock_multiplier)
{
  digitalWrite(ioreset,HIGH);digitalWrite(ioreset,LOW);
  SPI.transfer(chip,7,SPI_CONTINUE);
  SPI.transfer(chip,16,SPI_CONTINUE);
  SPI.transfer(chip,64 + clock_multiplier,SPI_CONTINUE);
  SPI.transfer(chip,0,SPI_CONTINUE);
  SPI.transfer(chip,32);
  digitalWrite(update,HIGH);digitalWrite(update,LOW);
}

void master_reset()
{
  digitalWrite(mreset,HIGH); delay(1); digitalWrite(mreset,LOW);
  digitalWrite(mreset,HIGH); delay(1); digitalWrite(mreset,LOW);
}

bool init(std::vector<String> comm, int comSize)
{
  if (comSize == 2)
  {
    // Master reset
    master_reset();
    
    // Control register
    clockmult = comm[1].toInt();
    control_register(cs1,clockmult);
    control_register(cs2,clockmult);
    sysclock = clockmult * refclock * (pow(10,6));
    Serial.println("INIT SUCCESS");
    return true;
  }
  else
  {
    Serial.println("ER:INIT needs exactly 1 paramter");
    flash(led1,250);
    return false;
  }
}

const String channels[4] = {"X1","X2","Y1","Y2"};
int get_channel(String channel)
{
  for (int i=0;i<4;i++)
  {
    if (channel==channels[i])
    {
      return i;
    }
  }
  return -1;
}

void get_(std::vector<String> comm, int comSize)
{
  if (comSize == 2)
  {
    int channel = get_channel(comm[1]);
    if (channel==-1)
    {
      Serial.println("ER:Invalic channel");
    }
    else
    {
      Serial.println(channels_[channel],4);
    }
  }
  else
  {
    Serial.println("ER:GET needs exactly 1 parameter");
  }
}

bool set(std::vector<String> comm, int comSize)
{ 
  if (comSize == 3)
  {
    int channel = get_channel(comm[1]);
    float value = comm[2].toFloat();
    
    //Make sure "value" is appropriate
    if (value>1)
    {
      Serial.println("ER:magnitude too high");
      return false;
    }
    if (value<0)
    {
      Serial.println("ER:magnitude negative");
      return false;
    }
    
    int chip;
    int inst;
    
    switch (channel)
    {
      case 0: // X1
      chip = cs1; // chip # 1
      inst = 8;   // channel x
      break;
      
      case 1: // X2
      chip = cs2; // chip # 2
      inst = 8;   // channel x
      break;
      
      case 2: // Y1
      chip = cs1; // chip # 1
      inst = 9;   // channel y
      break;
      
      case 3: // Y2
      chip = cs2; // chip # 2
      inst = 9;   // channel y
      break;
      
      default:
      Serial.println("ER:invalid channel");
      flash(led1,250);
      return false;
      break;
    }
    
    int vtrunc = floor(value * (pow(2,12) - 1));
    int b1 = vtrunc>>8;
    int b2 = vtrunc - (b1*256);
    
    digitalWrite(ioreset,HIGH);digitalWrite(ioreset,LOW);
    SPI.transfer(chip,inst            ,SPI_CONTINUE);
    SPI.transfer(chip,b1,SPI_CONTINUE);
    SPI.transfer(chip,b2             );
    
    float scale_set = (b1*256.0 + b2)/(4095.0);
    Serial.print("Success:channel ");
    Serial.print(comm[1]);
    Serial.print(" set to ");
    Serial.println(scale_set,4);
    channels_[channel] = scale_set;
    return true;
  }
  else
  {
    Serial.println("ER:SET needs exactly 2 parameters");
    flash(led1,250);
    return false;
  }
}

bool phase(std::vector<String> comm, int comSize)
{
  if (comSize == 3)
  {
    int chip;
    int which = comm[1].toInt();
    if (which == 1)
    {
      chip = cs1;
    }
    else if (which == 2)
    {
      chip = cs2;
    }
    else
    {
      return false;
    }
    
    float deg = comm[2].toFloat(); // Because modulo doesn't work on floats:
    int num = floor(deg/360);      // This is the equivalent of modulo,
    deg -= num*360;                // But it works on floats.
    
    int d = floor((deg/360)*(pow(2,14)-1));
    int byte1 = d >> 8         ;
    int byte2 = d - (byte1*256);
    digitalWrite(ioreset,HIGH);digitalWrite(ioreset,LOW);
    SPI.transfer(chip,0,SPI_CONTINUE);
    SPI.transfer(chip,byte1,SPI_CONTINUE);
    SPI.transfer(chip,byte2);
    float phs_set = 360.0 * (byte1*256.0 + byte2) / (16383.0);
    Serial.print("PHS(board ");
    Serial.print(which       );
    Serial.print(") set to " );
    Serial.println(phs_set,4 );
    if (chip == cs1)
    {
      phase_[0] = phs_set;
    }
    return true;
  }
  else
  {
    Serial.println("ER:PHS needs exactly 2 parameters");
    flash(led1,250);
    return false;
  }
}

void set_frequency(int chip, int instr, int b1, int b2, int b3, int b4, int b5, int b6)
{
  digitalWrite(ioreset,HIGH);digitalWrite(ioreset,LOW);
  SPI.transfer(chip,instr,SPI_CONTINUE);
  SPI.transfer(chip,b1   ,SPI_CONTINUE);
  SPI.transfer(chip,b2   ,SPI_CONTINUE);
  SPI.transfer(chip,b3   ,SPI_CONTINUE);
  SPI.transfer(chip,b4   ,SPI_CONTINUE);
  SPI.transfer(chip,b5   ,SPI_CONTINUE);
  SPI.transfer(chip,b6                );
}

bool frequency(std::vector<String> comm, int comSize)
{
  if (comSize == 2)
  {
    float freq = comm[1].toFloat();
    if (freq<=0)
    {
      Serial.println("ER:negative frequeny");
      return false;
    }
    double f = freq;
    f /= sysclock;
    if (f>1)
    {
      Serial.print("Frequency too high. It was set to ");
      Serial.print(freq);
      Serial.print(" whereas the maximum is currently ");
      Serial.println(sysclock);
      return false;
    }
    int b1 = floor(f * pow(2,8)); f = f*256 - b1;
    int b2 = floor(f * pow(2,8)); f = f*256 - b2;
    int b3 = floor(f * pow(2,8)); f = f*256 - b3;
    int b4 = floor(f * pow(2,8)); f = f*256 - b4;
    int b5 = floor(f * pow(2,8)); f = f*256 - b5;
    int b6 = floor(f * pow(2,8)); f = f*256 - b6;
    int instr = 2;
    set_frequency(cs1,instr,b1,b2,b3,b4,b5,b6);
    set_frequency(cs2,instr,b1,b2,b3,b4,b5,b6);
    double frq_set = (b1/pow(2,8.0) + b2/pow(2,16.0) + b3/pow(2,24.0) + b4/pow(2,32.0) + b5/pow(2,40.0) + b6/pow(2,48.0));
    Serial.print("FRQ set to ");
    Serial.print(frq_set*sysclock,4);
    Serial.println(" Hz");
    frequency_ = frq_set*sysclock;
  }
  else
  {
    Serial.println("ER:FRQ needs exactly 1 parameter");
    flash(led1,250);
    return false;
  }
}

void doUpdate()
{
  digitalWrite(update,HIGH);
  delay(0.01);
  digitalWrite(update,LOW);
}

void router(std::vector<String> comm, int comSize)
{
  int operation = indexOfOperation(comm[0]);
  bool success;
  switch(operation) // first piece determines action
  {
    case 0: // "NOP" or unrecognized command
    Serial.println("NOP");
    break;
    
    case 1: // "INIT": initialize box
    success = init(comm,comSize);
    break;
    
    case 2: // "SET": sets a magnitude for a pin
    success = set(comm,comSize);
    break;
    
    case 3:
    success = phase(comm,comSize);
    break;
    
    case 4:
    success = frequency(comm,comSize);
    break;
    
    case 5: // *IDN?
    Serial.println("ACBOX_DUAL_AD5764(UNNAMED1)");
    break;
    
    case 6: // *RDY?
    Serial.println("READY");
    break;
    
    case 7:
    master_reset();
    Serial.println("MASTER RESET SUCCESS");
    break;
    
    case 8:
    doUpdate();
    Serial.println("UPDATE SUCCESS");
    break;
    
    case 9: // GET
    get_(comm,comSize);
    break;
    
    case 10: // PHS?
    Serial.println(phase_[0],4);
    break;
    
    case 11: // FRQ?
    Serial.println(frequency_,4);
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
  int comSize = 0;
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
          comSize += 1;
          inByte = "";
        }
        else
        {
          inByte += received;
        }        
      }
    }
  router(comm,comSize);
  }
}

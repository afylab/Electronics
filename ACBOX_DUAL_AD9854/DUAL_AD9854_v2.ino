
/*
 how to use SPI originally based on:
 Example 34.1 - SPI bus demo using a Microchip MCP4162 digital potentiometer [http://bit.ly/iwDmnd]
 http://tronixstuff.com/tutorials > chapter 34 | CC by-sa-nc | John Boxall
 */

#include "SPI.h" // necessary library
int update=46; // I/O UD000
int mreset=9; // MRESET
int ioreset=34; //  A2
int cs1=4; // RDB
int cs2=52; // RDB
int chip; 
int led1=53;

void setup()
{
  Serial.begin(19200);
  pinMode(53,OUTPUT);
  pinMode(42, INPUT); 
  pinMode(2, OUTPUT); 
  pinMode(mreset, OUTPUT); 
  digitalWrite(mreset,LOW);
  pinMode(ioreset, OUTPUT); 
  digitalWrite(ioreset,LOW);
  pinMode(update, OUTPUT); 
  digitalWrite(update,LOW);
  SPI.begin(cs1); // wake up the SPI bus.
  SPI.begin(cs2); // wake up the SPI bus.
  SPI.setBitOrder(cs1,MSBFIRST); //correct order;
  SPI.setBitOrder(cs2,MSBFIRST); //correct order;
  SPI.setClockDivider(cs1,84);
  SPI.setClockDivider(cs2,84);
  SPI.setDataMode(cs1,SPI_MODE0); //Crucial!!!  Mode0 is *rising edge* transfer, which is compatible with AD9854.   
  SPI.setDataMode(cs2,SPI_MODE0); //Crucial!!!  Mode0 is *rising edge* transfer, which is compatible with AD9854.  
}

void commtest(byte DB[11])
{
  if (DB[0] == 255&&DB[1]==254&&DB[2]==253) //These are communication control bits, to make sure the arduino is getting the right bits.  
  {
    router(DB);
  }
  else //This is what it does if its not happy with the commtest bytes. It also provides a way to check whether msb/lsb is set correctly by allowing you to look at all the bytes being received on a scope. 
  {
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset,LOW); 
    SPI.transfer(DB[0]); SPI.transfer(DB[1]); SPI.transfer(DB[2]); SPI.transfer(DB[3]); SPI.transfer(DB[4]); SPI.transfer(DB[5]); SPI.transfer(DB[7]); SPI.transfer(DB[8]); SPI.transfer(DB[9]); SPI.transfer(DB[10]);
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset,LOW);
  }
}

  
void router(byte DB[11])
{
  switch (DB[3])
  {
    case 1:
    chip=cs1;
    setValue(DB);
    break;
    case 2:
    chip=cs2;
    setValue(DB);
    break;
    case 12:
    chip=cs1;
    setValue(DB);
    chip=cs2;
    setValue(DB);
    break;
    default:
    break;    
}
}

void setValue(byte DB[11])
{

switch (DB[4]) 
{
  case 0: // Phase adjust
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
    SPI.transfer(chip,DB[4],SPI_CONTINUE);  // Send instruction byte
    SPI.transfer(chip,DB[5],SPI_CONTINUE);   // Send data byte
    SPI.transfer(chip,DB[6]);  // Send data byte
    break;
  case 2: // Frequency tuning word
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
    SPI.transfer(chip,DB[4],SPI_CONTINUE);  // Send instruction byte
    SPI.transfer(chip,DB[5],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[6],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[7],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[8],SPI_CONTINUE);   // Send data byte
    SPI.transfer(chip,DB[9],SPI_CONTINUE); // Send data byte
    SPI.transfer(chip,DB[10]);  // Send data byte
     break;
     
  case 7: //Control register, should be 7 16 69 0 32
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
    SPI.transfer(chip,DB[4],SPI_CONTINUE);  // Send instruction byte
    SPI.transfer(chip,DB[5],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[6],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[7],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[8]);  // Send data byte
    digitalWrite(update, HIGH); digitalWrite(update, LOW); //Toggle I/O UD
     break;
  case 8: // I DAC multiplier
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
    SPI.transfer(chip,DB[4],SPI_CONTINUE);  // Send instruction byte
    SPI.transfer(chip,DB[5],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[6]);  // Send data byte
    break;
  case 9: // Q DAC multiplier
    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
    SPI.transfer(chip,DB[4],SPI_CONTINUE);  // Send instruction byte
    SPI.transfer(chip,DB[5],SPI_CONTINUE);  // Send data byte
    SPI.transfer(chip,DB[6]);  // Send data byte
     break;
  case 55:  //Master reset.  This code is arbitrarily chosen here
    digitalWrite(mreset, HIGH); delay(1); digitalWrite(mreset, LOW); delay(1); //Toggle MRSET up and down
    digitalWrite(mreset, HIGH); delay(1); digitalWrite(mreset, LOW); delay(1); //Toggle MRSET up and down
    digitalWrite(led1, HIGH); delay(100); digitalWrite(led1, LOW); delay(100);    digitalWrite(led1, HIGH); delay(100); digitalWrite(led1, LOW); delay(100); //Toggle MRSET up and down
  break;
  case 99:  //IO reset.  This code is arbitrarily chosen here
    digitalWrite(ioreset, HIGH); delay(1); digitalWrite(ioreset, LOW); delay(1); //Toggle IO RESET up and down
    digitalWrite(ioreset, HIGH); delay(1); digitalWrite(ioreset, LOW); delay(1); //Toggle IO RESET up and down    
    break; 
  case 101: 
  digitalWrite(update, HIGH); 
  delay(.01);
  digitalWrite(update, LOW); //Toggle I/O UD
  break;
//  case 128: // Read Phase - This is still not implemented fully
//    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
//    SPI.transfer(DB[4]); // send command byte.  00 (write-bar, 0) 010 (data register) 100 (select A-D).  decimal:  A = 16, B = 17, C = 18, D = 19, all = 20.
//    SPI.transfer(0);
//    SPI.transfer(0);
//    digitalWrite(chip1, HIGH); digitalWrite(chip2, HIGH); //Toggle chip select HIGH
//    break;
//  case 130: // Read Freq
//    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
//    digitalWrite(chip1, LOW); digitalWrite(chip2, LOW); //Toggle chip select LOW
//    SPI.transfer(DB[4]); // send command byte.  00 (write-bar, 0) 010 (data register) 100 (select A-D).  decimal:  A = 16, B = 17, C = 18, D = 19, all = 20.
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    digitalWrite(chip1, HIGH); digitalWrite(chip2, HIGH); //Toggle chip select HIGH
//    break;
//  case 135: // Read Control register
//    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
//    digitalWrite(chip1, LOW); digitalWrite(chip2, LOW); //Toggle chip select LOW
//    SPI.transfer(DB[4]); // send command byte.  00 (write-bar, 0) 010 (data register) 100 (select A-D).  decimal:  A = 16, B = 17, C = 18, D = 19, all = 20.
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    SPI.transfer(0);
//    digitalWrite(chip1, HIGH); digitalWrite(chip2, HIGH); //Toggle chip select HIGH
//    break;
//  case 136: // Read I dac
//    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
//    digitalWrite(chip1, LOW); digitalWrite(chip2, LOW); //Toggle chip select LOW
//    SPI.transfer(DB[4]); // send command byte.  00 (write-bar, 0) 010 (data register) 100 (select A-D).  decimal:  A = 16, B = 17, C = 18, D = 19, all = 20.
//    SPI.transfer(0);
//    SPI.transfer(0);
//    digitalWrite(chip1, HIGH); digitalWrite(chip2, HIGH); //Toggle chip select HIGH
//    break;
//  case 137: // Read Q dac
//    digitalWrite(ioreset, HIGH); digitalWrite(ioreset, LOW); //IORESET BEFORE WRITE
//    digitalWrite(chip1, LOW); digitalWrite(chip2, LOW); //Toggle chip select LOW
//    SPI.transfer(DB[4]); // send command byte.  00 (write-bar, 0) 010 (data register) 100 (select A-D).  decimal:  A = 16, B = 17, C = 18, D = 19, all = 20.
//    SPI.transfer(0);
//    SPI.transfer(0);
//    digitalWrite(chip1, HIGH); digitalWrite(chip2, HIGH); //Toggle chip select HIGH
//    break;
  default:
  break;}

}
void loop()
{
  while ( Serial.available()<10) // wait until all data bytes are avaialable
  {digitalWrite(led1,HIGH);
  }
if (Serial.available()) // wait until all three data bytes are avaialable
    {
     byte bytes[11];
      for (int i=0; i<11; i++) {
      bytes[i] = Serial.read();
      delay(2); //Why do you need this? I don't remember but it is necessary
    }
    digitalWrite(led1,LOW);
    delay(100);
        commtest(bytes);
    }

}

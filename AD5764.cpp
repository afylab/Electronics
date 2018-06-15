//
//	Class for Analog AD5764  16-Bit DAC:
//	4-Channel, ±10 V Input Range
//
#include "Arduino.h"
#include "AD5764.h"
#include "SPI.h" 

AD5764::AD5764(int spi_pin, int ldac_pin)
{
	spi = spi_pin;
	ldac = ldac_pin;
};

bool AD5764::begin(void)
{
	pinMode(ldac, OUTPUT);   
	digitalWrite(ldac, HIGH);
	SPI.begin(spi);
	SPI.setBitOrder(spi, MSBFIRST); //correct order for AD5764.
	SPI.setClockDivider(spi, 4); // Maximum 21 Mhz for AD5764
	SPI.setDataMode(spi, SPI_MODE1); //This should be 1 for the AD5764
};

float AD5764::dacDataSend(int ch, float voltage)
{
	byte b1, b2;

	floatToTwoByte(voltage, b1, b2);
	digitalWrite(data, HIGH);
	SPI.transfer(spi, 16 + ch, SPI_CONTINUE); // Indicates to DAC to write channel 'ch' in the data register 
	SPI.transfer(spi, b1, SPI_CONTINUE);   // writes first byte
	SPI.transfer(spi, b2);                // writes second byte

	digitalWrite(ldac,LOW);
	digitalWrite(ldac,HIGH);
	digitalWrite(data, LOW);

	return twoByteToFloat(b1, b2);
};

void AD5764::intToTwoByte(int s, byte& MSB, byte& LSB)
{
    MSB = ((byte)((s >> 8) & 0xFF));  // shift and mask to get the MSB
    LSB = ((byte)(s & 0xFF));  // mask to get the LSB
}

// Prepare a voltage for write to data buffer assuming 2s compliment
// see Table 8 of refernce. 2s compliment is set by LK9 jumper
void AD5764::voltageToTwoByte(float voltage, byte& MSB, byte& LSB)
{
	int decimal;
  	if (voltage > DACfullScale || voltage < -1*DACfullScale)
  {
    MSB = 128;  // I think Carlos confused Table 8 and 9; This sets -10V, should be MSB=0
    LSB = 0;
    raiseError();
  }  
  else if (voltage >= 0) // First bit is 0 for positive, 1 for negative; Unisigned 15bits to encode voltage
  {
    decimal = voltage*32767/DAC_FULL_SCALE;  
  }
  else
  {
    decimal = voltage*32768/DAC_FULL_SCALE + 65536;  
  }
  intToTwoByte(decimal, MSB, LSB);
}
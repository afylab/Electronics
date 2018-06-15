//
//	Class for Analog AD5764  16-Bit DAC:
//	4-Channel, ±10 V Input Range
//
#ifndef AD5764_h
#define AD5764_h

#include "Arduino.h"
#include "SPI.h" 

class AD5764
{
public:
	AD5764(int spi_pin, int ldac_pin);
	bool begin(void);
	float dacDataSend(int channel, float voltage);

private:
	int spi;
	int ldac ;
	float DACfullScale = 10;
	int timeUnit = 0;  // 0 - us, 1 - ms
	float channelOffset[4] = {0, 0, 0, 0};  // output channel offsets
	float channelGain[4] = {1, 1, 1,  1};   // output channel gain corrections

	void voltageToTwoByte(float voltage, byte& MSB, byte& LSB);
	void intToTwoByte(int value, byte&  MSB, byte& LSB);
	float twoByteToVoltage(byte MSB, byte LSB);

	void raiseError();

};
#endif
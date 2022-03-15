/*
 *  adc.h
 *
 *  Description: Class to implement an ADC on Raspberry Pi SPI device 0
 *
 *  Interface:
 *    - GetVoltage(): Read the current voltage of the battery
 *
 *  Created on: Mar 2, 2017
 *      Author: jeff
 */

#ifndef INCLUDE_ADC_H_
#define INCLUDE_ADC_H_

#include "dp_events.h"

#define SPI_DEV_0 "/dev/spidev0.0"

class ADC : public DP::GenericSensor
{
private:
	const char *spiDevId;
	unsigned digitalCodes[8];

	int InitSPI(const char *dev);

protected:
    // get the actual digital code value from the ADC device
	int GetDigitalCode(unsigned channel)
	{
		return digitalCodes[channel];
	}
	
	// event handler for the ADC object
	void Routine();

public:
	ADC(unsigned _period);
	virtual ~ADC()
	{}
	float GetVoltage(unsigned channel)
	{
		return ((digitalCodes[channel] * 3.3) / 1024);
	}
};

#endif /* INCLUDE_ADC_H_ */

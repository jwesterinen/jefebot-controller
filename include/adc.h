/*
 * adc.h
 *
 *  Created on: Mar 2, 2017
 *      Author: jeff
 */

#ifndef INCLUDE_ADC_H_
#define INCLUDE_ADC_H_

#include "framework.h"

#define SPI_DEV_0 "/dev/spidev0.0"

class ADC : public GenericSensor
{
private:
	const char *spiDevId;
	unsigned digitalCodes[8];

	int InitSPI(const char *dev);

protected:
	int GetDigitalCode(unsigned channel)
	{
		return digitalCodes[channel];
	}
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

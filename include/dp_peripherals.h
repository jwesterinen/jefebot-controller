/*
 * dp_peripheral.h
 *
 *  Created on: Mar 8, 2017
 *      Author: jeff
 */

#ifndef DP_PERIPHERALS_H_
#define DP_PERIPHERALS_H_

#include <sstream>

// local implementation of the std::to_string function
template <typename T>
const std::string ToString(T val)
{
	std::stringstream stream;
	stream << val;
	return stream.str();
}

// DP peripheral base class
class DP_Peripheral
{
protected:
	int cmdfd;
	std::string id;

	void WriteConfig(std::string resource, const std::string& arg);
	void WriteCommand(std::string command, std::string arg = "");

public:
	DP_Peripheral(Framework& framework, const char* _id) : cmdfd(framework.cmdfd), id(_id)
	{}
	virtual ~DP_Peripheral()
	{}
};

// DP sensor base class
class DP_Sensor : public DP_Peripheral, public SelectableSensor
{
protected:
	std::string dataRsc;

	void ReadResponse(char* buffer, int bufSize);
	void StartDataStream();
	virtual void Handler() = 0;

public:
	DP_Sensor(Framework& framework, const char* id, const char* _dataRsc) : DP_Peripheral(framework, id), dataRsc(_dataRsc)
	{}
	virtual ~DP_Sensor()
	{}
};

// DP actuator base class interface
class DP_Actuator : public DP_Peripheral
{
public:
	DP_Actuator(Framework& framework, const char* id) : DP_Peripheral(framework, id)
	{}
	virtual ~DP_Actuator()
	{}
};

// BB4IO peripheral
class DP_Bb4io : public DP_Sensor
{
private:
	unsigned buttons;

protected:
	void SetLeds(unsigned char pattern);
	void Handler();

public:
	// OR these together to combine buttons
	enum SWITCHES {S1 = 1, S2 = 2, S3 = 4};

	DP_Bb4io(Framework& framework) : DP_Sensor(framework, "bb4io", "buttons"), buttons(0)
	{}
	virtual ~DP_Bb4io()
	{}
	bool IsButtonPressed(int buttonId)
	{
		return (buttons & buttonId);
	}
};

// PING4 peripheral
class DP_Ping4 : public DP_Sensor
{
private:
	unsigned distances[4];

protected:
	unsigned GetDistance(unsigned channelId);
	void Handler();

public:
	// TODO: fill in the actual ranges for a Ping device
	const static unsigned MinRange = 0;
	const static unsigned MaxRange = 5000;
	// OR these together to form an enable sensor pattern
	enum SENSORS {SENSOR_0 = 1, SENSOR_1 = 2, SENSOR_2 = 4, SENSOR_3 = 8};

	DP_Ping4(Framework& framework, const char* id) : DP_Sensor(framework, id, "distance")
	{
		distances[0] = distances[1] = distances[2] = distances[3] = 0;
	}
	virtual ~DP_Ping4()
	{}
	void Enable(unsigned sensorPattern)
	{
	    return WriteConfig("enable", ToString(sensorPattern));
	}
};

// count4 peripheral
class DP_Count4 : public DP_Sensor
{
private:
    unsigned counts[4];
    float intervals[4];

protected:
    unsigned GetCount(unsigned channelId)
    {
    	return counts[channelId];
    }
    float GetInterval(unsigned channelId)
    {
    	return intervals[channelId];
    }
	void Handler();

public:
    enum EDGE_CONFIG {DISABLE_EDGE = 0, LEADING_EDGE = 1, TRAILING_EDGE = 2, BOTH_EDGES = 3};

    DP_Count4(Framework& framework, const char* id) : DP_Sensor(framework, id, "counts")
    {
    	for (int i = 0; i < 4; ++i)
    	{
    		counts[i] = 0;
    		intervals[i] = 0.0;
    	}
    }
	virtual ~DP_Count4()
	{}
	void SetUpdateRate(unsigned rate);
	void SetEdges(unsigned setting1, unsigned setting2, unsigned setting3, unsigned setting4);
};

// PING4 peripheral
class DP_Adc812 : public DP_Sensor
{
private:
	unsigned samples[8];

protected:
	unsigned GetSample_mV(unsigned inputId)
	{
		return samples[inputId];
	}
	void Handler();

public:
	enum CHANNELS {CHANNEL_1 = 0, CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6, CHANNEL_7, CHANNEL_8};
	// OR these together to form an enable pair pattern
	enum DIFF_PAIRS {NO_PAIRS = 0, PAIR_1 = 1, PAIR_2 = 2, PAIR_3 = 4, PAIR_4 = 8};

	DP_Adc812(Framework& framework, const char* id) : DP_Sensor(framework, id, "samples")
	{
		for (int i = 0; i < 8; ++i)
			samples[i] = 0;
	}
	virtual ~DP_Adc812()
	{}
	void Config(unsigned period, DIFF_PAIRS differentialPairs);
};

// DC2 peripheral

// TODO: implement missing configurators
class DP_DC2 : public DP_Actuator
{
public:
	enum MODE {BREAK = 'b', FORWARD = 'f', REVERSE = 'r', COAST = 'c'};

	DP_DC2(Framework& framework, const char* id) : DP_Actuator(framework, id)
	{}
	virtual ~DP_DC2()
	{}
	void SetMode0(char mode)
	{
		return WriteConfig("mode0", ToString(mode));
	}
	void SetMode1(char mode)
	{
		return WriteConfig("mode1", ToString(mode));
	}
	void SetPower0(float power)
	{
		return WriteConfig("power0", ToString(power));
	}
	void SetPower1(float power)
	{
		return WriteConfig("power1", ToString(power));
	}
	void SetPWMfrequency(unsigned freqInHz)
	{
		return WriteConfig("pwm_frequency", ToString(freqInHz));
	}
	/*
	slow_start0, slow_start1 : Acceleration curve
	The acceleration curve is a list of ten space separated integers
	representing the time (in milliseconds) between each 10 percent
	increase from 0 to 100 percent ON time.  The following string
	shows how to specify a slow start that takes half a second to
	get to 50 percent power and three-quarters of a second to get
	from zero to full power:
		dpset dc2 slow_start0 100 100 100 100 100 50 50 50 50 50
	The default value for slow_start is off to full power in zero
	milliseconds.  That is, the default is to turn off slow start.
	*/
	void SetSlow_start0()
	{
		return;
	}
	void SetSlow_start1()
	{
		return;
	}
	/*
	slow_stop0, slow_stop1 : Deceleration curve.
	The deceleration curve is a list of ten space separated integers
	representing the time (in milliseconds) between each 10 percent
	decrease from 100 to 0 percent ON time.  The following string
	shows a slow stop that takes half a second to get to 50% power
	and three-quarters of a second to get from full power to zero:
	    dpset cd2 slow_stop0 100 100 100 100 100 50 50 50 50 50
	The default value for slow_stop is full on to full off in zero
	milliseconds.  That is, the default is to turn off slow stop.
	*/
	void SetSlow_stop0()
	{
		return;
	}
	void SetSlow_stop1()
	{
		return;
	}
	void SetWatchdog(unsigned timeout)
	{
		return WriteConfig("watchdog", ToString(timeout));
	}
};

#endif /* DP_PERIPHERALS_H_ */

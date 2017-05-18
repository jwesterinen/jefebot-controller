/*
 * peripherals.h
 *
 *  Created on: Mar 20, 2017
 *      Author: jeff
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "framework.h"
#include "dp_peripherals.h"
#include "adc.h"

// DP peripheral list -- this must agree with the output of dplist
#define BB4IO	"1"		// The buttons and LEDs on the Baseboard
#define DC2		"2"		// Dual DC motor controller
#define COUNT4 	"3"		// Quad Event counter
#define ADC812	"4"   	// Octal 12-bit Analog-to-Digital converter
#define WS28	"5"	   	// Quad WS2812 LED driver
#define DC2_2	"6"		// Dual DC motor controller
#define SERVO4	"7"		// Four servo control pins
#define IRIO	"8"   	// Consumser IR receiver and transmitter
#define TIF		"9"   	// Text Interface
#define PING4 	"10"  	// Quad interface to a Parallax Ping)))

class UserInterface : public DP_Bb4io
{
public:
	enum BUTTONS {BUTTON1 = S1, BUTTON2 = S2, BUTTON3 = S3};

	UserInterface(Framework& framework);
	void Display(unsigned char pattern)
	{
		SetLeds(pattern);
	}
	bool IsButtonPressed(enum BUTTONS buttonId)
	{
		return DP_Bb4io::IsButtonPressed(buttonId);
	}
};

class Locomotive : public DP_Count4, public DP_DC2
{
private:
	const static unsigned Count4Period = 50;
	const static unsigned DC2WatchdogTimeout = 1500;
	const static float MinSpeed = 20.0;
	const static float MaxSpeed = 100.0;
	const static unsigned RadiansPerTick = 0;
	enum DIRECTION {STOP, MOVE_FORWARD, MOVE_REVERSE, SPIN_CW, SPIN_CCW} direction;
	float defaultSpeed;
	int ticks[2];			// total accumulated count -- must be signed, +/- -> fwd/rev
	char modes[2];
	float speeds[2];

	float TicksToRadians(unsigned ticks);
	float TicksToCm(unsigned ticks);

protected:
	void Handler();

public:
	enum SIDE {LEFT = 0, RIGHT};

	Locomotive(Framework& framework, float defaultSpeed);
	~Locomotive()
	{
		//delete motors;
	}
	int GetTicks(int index)
	{
		return ticks[index];
	}
	float GetMode(int index)
	{
		return modes[index];
	}
	float GetSpeed(int index)
	{
		return speeds[index];
	}
	void SetMode(int index, char mode);
	void SetSpeed(int index, float speed);
	void Stop();
	void MoveForward(unsigned distance);
	void MoveReverse(unsigned distance);
	void SpinCW(float angle);
	void SpinCCW(float andle);
	void MoveAngle(float radians);
	// TODO: what metric to use for distance, cm or in
	void MoveDistance(unsigned cm);
};

/*
 *  The RangeSensor class is based on a DP_Ping4 peripheral with a single sensor, SENSOR_0.
 */
class SinglePingRangeSensor : public DP_Ping4
{
private:
	unsigned innerLimit;
	unsigned outerLimit;

public:
	SinglePingRangeSensor(Framework& framework, int _innerLimit, int _outerLimit);
	unsigned GetDistance()
	{
		return DP_Ping4::GetDistance(SENSOR_0);
	}
	bool AtObject()
	{
	    return (GetDistance() < innerLimit);
	}
	bool DetectObject(unsigned* pDistance)
	{
	    return ((*pDistance = GetDistance()) < outerLimit);
	}
};

// edge detector based on 3 Sharp GP2Y0A21YK0F distance sensors
class EdgeDetector : public DP_Adc812
{
private:
	const static unsigned Period = 50;
	unsigned edgeLimits[3];

public:
#ifdef USE_DISTANCE_NOT_VOLTAGE
	const static unsigned MinEdgeRange = 10;
	const static unsigned MaxEdgeRange = 80;
#else
	// TODO: use actual sensor min and max values
	const static unsigned MinEdgeRange = 0;
	const static unsigned MaxEdgeRange = 5000;
#endif
	enum EDGE_SENSORS {LEFT = CHANNEL_1, FRONT = CHANNEL_2, RIGHT = CHANNEL_3};

	EdgeDetector(Framework& framework, unsigned nominalEdgeLimit);
	bool AtAnyEdge()
	{
		return AtEdge(LEFT) || AtEdge(FRONT) || AtEdge(RIGHT);
	}
	// TODO: change from using voltage to distance for values and limits
#ifdef USE_DISTANCE_NOT_VOLTAGE
	bool AtEdge(enum EDGE_SENSORS sensorId)
	{
	    return (GetEdgeSensorDistance_cm(sensorId) < edgeLimits[sensorId]);
	}
	unsigned GetEdgeSensorDistance_cm(EDGE_SENSORS sensorId)
	{
		// from the Sharp GP2Y0A21YK0F datasheet
		return (27 / (GetSample_mV(sensorId) / 1000));
	}
#else
	bool AtEdge(enum EDGE_SENSORS sensorId)
	{
	    return (GetSample_mV(sensorId) < edgeLimits[sensorId]);
	}
	unsigned GetEdgeSensorValue(EDGE_SENSORS sensorId)
	{
		// return the sensor voltage in mV
		return GetSample_mV(sensorId);
	}
#endif
};

/*
 * This class is implemented with an ADC being handled at 50mS
 */
class VoltMeter : public ADC
{
public:
	VoltMeter(Framework& framework) : ADC(50)
	{
		framework.Register(this);
	}
};

#endif /* PERIPHERALS_H_ */

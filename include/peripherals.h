/*
 * peripherals.h
 *
 *  Created on: Mar 20, 2017
 *      Author: jeff
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include <dp_adc812.h>
#include <dp_bb4io.h>
#include <dp_count4.h>
#include <dp_dc2.h>
#include <dp_ping4.h>
#include "adc.h"

#define DO_NOT_USE_RANDOM_VALUES

// DP peripheral list -- this must agree with the output of dplist
#define BB4IO_IDX	"1"		// The buttons and LEDs on the Baseboard
#define DC2_IDX		"2"		// Dual DC motor controller
#define COUNT4_IDX 	"3"		// Quad Event counter
#define ADC812_IDX	"4"   	// Octal 12-bit Analog-to-Digital converter
#define WS28_IDX	"5"	   	// Quad WS2812 LED driver
#define DC2_2_IDX	"6"		// Dual DC motor controller
#define SERVO4_IDX	"7"		// Four servo control pins
#define IRIO_IDX	"8"   	// Consumser IR receiver and transmitter
#define TIF_IDX		"9"   	// Text Interface
#define PING4_IDX 	"10"  	// Quad interface to a Parallax Ping)))

class UserInterface : public DP::BB4IO
{
public:
	enum BUTTONS {BUTTON1 = S1, BUTTON2 = S2, BUTTON3 = S3};

	UserInterface(DP::EventContext& evtCtx);
	void Display(unsigned char pattern)
	{
		SetLeds(pattern);
	}
	bool IsButtonPressed(enum BUTTONS buttonId)
	{
		return DP::BB4IO::IsButtonPressed(buttonId);
	}
};

class Locomotive : public DP::COUNT4, public DP::DC2
{
private:
	const static unsigned Count4Period = 50;
	const static unsigned WatchdogTimeout = 0;
	const static float MinSpeed = 20.0;
	const static float MaxSpeed = 100.0;
	const static float MaxVelocityErr = 5.0;
	const static unsigned TicksPerCM = 2;
	const static unsigned TicksPerRadian = 14;


    // TODO: tweak, tweak, tweak !!!
	// PID controller gains
	const static float Kp = 0.02;
	const static float Ki = 0.0;
	const static float Kd = 0.0;

	enum DIRECTION {STOP, MOVE_FORWARD, MOVE_REVERSE, SPIN_CW, SPIN_CCW} direction;
	float defaultSpeed;
	int ticks[2];			// total accumulated count -- must be signed, +/- -> fwd/rev
	char modes[2];
	float powers[2];

	void MoveDistance(unsigned distanceInCm);
	float TicksToCm(unsigned ticks);
	void MoveAngle(float angleInRadians);
	float TicksToRadians(unsigned ticks);

protected:
	void Handler();

public:
	enum SIDE {LEFT = 0, RIGHT};

	Locomotive(DP::EventContext& evtCtx, float defaultSpeed);
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
	float GetPower(int index)
	{
		return powers[index];
	}
	void ClearTicks();
	void SetMode(char modeL, char modeR);
	void SetPower(float powerL, float powerR);
	void Stop();
	void MoveForward();
	void MoveReverse();
	void SpinCW();
	void SpinCCW();
	bool HasMovedDistance(unsigned distanceInCm, unsigned* curDistance = 0);
	bool HasTurnedAngle(float angleInRadians, float* curAngle = 0);
};

/*
 *  The RangeSensor class is based on a DP_Ping4 peripheral with a single sensor, SENSOR_0.
 */
class SinglePingRangeSensor : public DP::PING4
{
private:
	unsigned innerLimit;
	unsigned outerLimit;

public:
	SinglePingRangeSensor(DP::EventContext& evtCtx, int _innerLimit, int _outerLimit);
	unsigned GetDistance()
	{
		return DP::PING4::GetDistance(SENSOR_0);
	}
	bool AtObject()
	{
	    return (GetDistance() < innerLimit);
	}
	bool DetectObject(unsigned limit, unsigned* pDistance)
	{
		if (limit == 0)
		{
			limit = outerLimit;
		}
	    return ((*pDistance = GetDistance()) < limit);
	}
};

// edge detector based on 3 Sharp GP2Y0A21YK0F distance sensors
class EdgeDetector : public DP::ADC812
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

	EdgeDetector(DP::EventContext& evtCtx, unsigned nominalEdgeLimit);
	bool AtAnyEdge(enum EDGE_SENSORS* pEdge = 0);
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
	VoltMeter(DP::EventContext& evtCtx) : ADC(50)
	{
		evtCtx.Register(this);
	}
};

#endif /* PERIPHERALS_H_ */

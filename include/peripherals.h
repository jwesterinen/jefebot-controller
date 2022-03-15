/*
 *  peripherals.h
 *
 *  Description: a set of peripheral classes specialized from the DP peripheral classes
 *  for jefebot.
 *
 *  All classes and interfaces are described below.
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

/*
 * class to control the LEDs and buttons on the BBIO4 board
 */
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

/*
 * combo class to implement a dual motor controller and accept the ticks returned
 * from each motor to keep track of the current position of the bot
 */
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

protected:
	void Handler();

public:
	enum SIDE {LEFT = 0, RIGHT};

	Locomotive(DP::EventContext& evtCtx, float defaultSpeed);
	~Locomotive()
	{
		//delete motors;
	}
	
	// return the current tick count of the motors
	int GetTicks(int index)
	{
		return ticks[index];
	}
	
	// return the current mode, i.e. forward/reverse, of the motors
	float GetMode(int index)
	{
		return modes[index];
	}
	
	// return the current power value, i.e. speed, of the motors
	float GetPower(int index)
	{
		return powers[index];
	}
	
	// clear all motor ticks
	void ClearTicks();
	
	// set the mode, power of the motors
	void SetMode(char modeL, char modeR);
	void SetPower(float powerL, float powerR);
	
	// halt the movement of the motors
	void Stop();
	
	// describe which direction to move, this is used in conjunction with the
	// HasMovedDistance() function to perform a linear movement
	void MoveForward();
	void MoveReverse();
	
	// describe which angle to turn, this is used in conjunction with the
	// HasTurnedAngle() function to perform an angular movement
	void SpinCW();
	void SpinCCW();
	
	// flag to signify that the requested distance moved has been achieved
	bool HasMovedDistance(unsigned distanceInCm, unsigned* curDistance = 0);
	
	// flag to signify that the requested angle turned has been achieved
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
	
	// get the currently sensed distance
	unsigned GetDistance()
	{
		return DP::PING4::GetDistance(SENSOR_0);
	}
	
	// flag to signify that the bot is next to an object
	bool AtObject()
	{
	    return (GetDistance() < innerLimit);
	}
	
	// return the distance from an object within a given limit
	bool DetectObject(unsigned limit, unsigned* pDistance)
	{
		if (limit == 0)
		{
			limit = outerLimit;
		}
	    return ((*pDistance = GetDistance()) < limit);
	}
};

/*
 * combination 3-edge detector based on 3 Sharp GP2Y0A21YK0F distance sensors
 */
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
	
	// flag to signify that some edge has been detected
	bool AtAnyEdge(enum EDGE_SENSORS* pEdge = 0);
	// TODO: change from using voltage to distance for values and limits
	
#ifdef USE_DISTANCE_NOT_VOLTAGE
    // flag to signify that a specific edge has been detected
	bool AtEdge(enum EDGE_SENSORS sensorId)
	{
	    return (GetEdgeSensorDistance_cm(sensorId) < edgeLimits[sensorId]);
	}
	
	// return the sensed distance of an edge detector
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
 * a volt meter class implemented with an ADC being handled at 50mS
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

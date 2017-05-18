/*
 * peripherals.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: jeff
 */

#ifndef DO_NOT_USE_RANDOM_VALUES
#include <unistd.h>
#endif
#include "peripherals.h"
#include <cstdio>
#include <cassert>

UserInterface::UserInterface(Framework& framework) : DP_Bb4io(framework)
{
	framework.Register(this);
	StartDataStream();
}

Locomotive::Locomotive(Framework& framework, float _defaultSpeed) :
	DP_Count4(framework, COUNT4), DP_DC2(framework, DC2), direction(STOP), defaultSpeed(_defaultSpeed)
{
	ticks[0] = ticks[1] = 0;
	modes[0] = modes[1] = DP_DC2::BREAK;
	speeds[0] = speeds[1] = 0.0;

	if (MinSpeed > defaultSpeed || defaultSpeed > MaxSpeed)
	{
		throw FrameworkException("Locomotive speed", ERR_PARAMS);
	}

	// register and configure the DP Count4 peripheral
	framework.Register(this);
	SetUpdateRate(Count4Period);
	SetEdges(BOTH_EDGES, BOTH_EDGES, DISABLE_EDGE, DISABLE_EDGE);
	StartDataStream();

	// configure the DP DC2 peripheral
#ifdef SET_WATCHDOG
	SetWatchdog(WatchdogTimeout);
#endif
}

void Locomotive::Stop()
{
    direction = STOP;
    SetMode(LEFT, BREAK);
    SetMode(RIGHT, BREAK);
    SetSpeed(LEFT, defaultSpeed);
    SetSpeed(RIGHT, defaultSpeed);
}

// move indefinitely if distance is 0
void Locomotive::MoveForward(unsigned distance)
{
    direction = MOVE_FORWARD;
    SetMode(LEFT, FORWARD);
    SetMode(RIGHT, FORWARD);
    if (distance > 0)
    {
    	MoveDistance(distance);
    	Stop();
    }
}

// move indefinitely if distance is 0
void Locomotive::MoveReverse(unsigned distance)
{
    direction = MOVE_REVERSE;
    SetMode(LEFT, REVERSE);
    SetMode(RIGHT, REVERSE);
    if (distance > 0)
    {
    	MoveDistance(distance);
    	Stop();
    }
}

// spin indefinitely if distance is 0
void Locomotive::SpinCW(float angle)
{
    direction = SPIN_CW;
    SetMode(LEFT, FORWARD);
    SetMode(RIGHT, REVERSE);
    if (angle > 0)
    {
    	MoveAngle(angle);
    	Stop();
    }
}

// spin indefinitely if distance is 0
void Locomotive::SpinCCW(float angle)
{
    direction = SPIN_CCW;
    SetMode(LEFT, REVERSE);
    SetMode(RIGHT, FORWARD);
    if (angle > 0)
    {
    	MoveAngle(angle);
    	Stop();
    }
}

void Locomotive::SetMode(int index, char mode)
{
	modes[index] = mode;
	if (index == LEFT)
		return SetMode0(mode);
	else
		return SetMode1(mode);
}

void Locomotive::SetSpeed(int index, float speed)
{
	if (MinSpeed > speed || speed > MaxSpeed)
    {
    	throw FrameworkException("Locomotive speed", ERR_PARAMS);
    }
    if (speed != speeds[index])
    {
        speeds[index] = speed;
        if (index == LEFT)
        	SetSpeed0(speed);
        else
        	SetSpeed1(speed);
    }
}

float Locomotive::TicksToRadians(unsigned ticks)
{
	// TODO: use the real equation
	return 0;
}

void Locomotive::MoveAngle(float radians)
{
#ifdef DO_NOT_USE_RANDOM_VALUES
	if (radians > 0.0)
	{
		volatile unsigned ticks;
		volatile unsigned beginTicks = GetTicks(LEFT);
		do
		{
			ticks = GetTicks(LEFT);
		}
		while (TicksToRadians(ticks - beginTicks) < radians);
	}
#else
	usleep(radians);
#endif
}

float Locomotive::TicksToCm(unsigned ticks)
{
	// TODO: use the real equation
	return 0;
}

void Locomotive::MoveDistance(unsigned cm)
{
#ifdef DO_NOT_USE_RANDOM_VALUES
	if (distance > 0)
	{
		volatile unsigned ticks;
		volatile unsigned beginTicks = GetTicks(LEFT);
		do
		{
			ticks = GetTicks(LEFT);
		}
		while (TicksToDistance(ticks - beginTicks) < distance);
	}
#else
	usleep(cm);
#endif
}

#define P_LOOP
void Locomotive::Handler()
{
	// call the counter's handler to get the current values
	DP_Count4::Handler();

    // accumulate the ticks
    ticks[LEFT] += (GetMode(LEFT) == FORWARD) ? GetCount(LEFT) : -GetCount(LEFT);
    ticks[RIGHT] += (GetMode(RIGHT) == FORWARD) ? GetCount(RIGHT) : -GetCount(RIGHT);

#ifdef P_LOOP
    // PID controller
    // TODO: tweak, tweak, tweak !!!
    float Kp = 0.01;

    if (direction == MOVE_FORWARD && GetInterval(LEFT) != 0 && GetInterval(RIGHT) != 0)
    {
    	float vl = GetCount(LEFT) / GetInterval(LEFT);
		float vr = GetCount(RIGHT) / GetInterval(RIGHT);
		float err = (vl > vr) ? vl - vr : vr - vl;
printf("PID: err = %f\n", err);
		float P = Kp * err;
		float speedL = GetSpeed(LEFT);
		float speedR = GetSpeed(RIGHT);
		if (vl > vr)
		{
			SetSpeed(LEFT, speedL - ((P/2) * speedL));
			SetSpeed(RIGHT, speedR + ((P/2) * speedR));
		}
		else
		{
			SetSpeed(LEFT, speedL + ((P/2) * speedL));
			SetSpeed(RIGHT, speedR - ((P/2) * speedR));
		}
    }
#else
    // balance the motor speeds
    if (direction == MOVE_FORWARD)
    {
    	float newSpeed = GetSpeed(LEFT);
    	newSpeed = (GetCount(LEFT) > GetCount(RIGHT)) ? newSpeed - 0.1 : newSpeed + 0.1;
		SetSpeed(LEFT, newSpeed);
    }
#endif
}

SinglePingRangeSensor::SinglePingRangeSensor(Framework& framework, int _innerLimit, int _outerLimit) :
	DP_Ping4(framework, PING4), innerLimit(_innerLimit), outerLimit(_outerLimit)
{
	if (
		MinRange > innerLimit || innerLimit > MaxRange ||
		MinRange > outerLimit || outerLimit > MaxRange
	)
    {
    	throw FrameworkException("SinglePingRangeSensor", ERR_PARAMS);
    }

	framework.Register(this);
	Enable(SENSOR_0);
	StartDataStream();
}

// TODO: change class name to the specific brand/type of sensor
EdgeDetector::EdgeDetector(Framework& framework, unsigned nominalEdgeLimit) : DP_Adc812(framework, ADC812)
{
	if (MinEdgeRange > nominalEdgeLimit || nominalEdgeLimit > MaxEdgeRange)
    {
    	throw FrameworkException("SinglePingRangeSensor", ERR_PARAMS);
    }
	for (int i = 0; i < 3; ++i)
	{
		edgeLimits[i] = nominalEdgeLimit;
	}

	framework.Register(this);
	Config(Period, NO_PAIRS);
	StartDataStream();
}


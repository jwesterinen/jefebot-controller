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
	powers[0] = powers[1] = 0.0;

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
    SetMode(BREAK, BREAK);
    SetPower(defaultSpeed, defaultSpeed);
}

// move indefinitely if distance is 0
void Locomotive::MoveForward(unsigned distance)
{
    direction = MOVE_FORWARD;
    SetMode(FORWARD, FORWARD);
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
    SetMode(REVERSE, REVERSE);
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
    SetMode(FORWARD, REVERSE);
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
    SetMode(REVERSE, FORWARD);
    if (angle > 0)
    {
    	MoveAngle(angle);
    	Stop();
    }
}

void Locomotive::SetMode(char modeL, char modeR)
{
	SetMode0(modes[LEFT] = modeL);
	SetMode1(modes[RIGHT] = modeR);
}

void Locomotive::SetPower(float powerL, float powerR)
{
	if ((MinSpeed <= powerL && powerL <= MaxSpeed) && (MinSpeed <= powerR || powerR <= MaxSpeed))
    {
	    if (powerL != powers[LEFT])
	    {
	        SetPower0(powers[LEFT] = powerL);
	    }
	    if (powerR != powers[RIGHT])
	    {
	        SetPower1(powers[RIGHT] = powerR);
	    }
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

    if (direction == MOVE_FORWARD && GetInterval(LEFT) != 0 && GetInterval(RIGHT) != 0)
    {
    	// calculate the velocity of each motor
		unsigned countL = GetCount(LEFT);
		float intvlL = GetInterval(LEFT);
    	float vl = countL / intvlL;
		unsigned countR = GetCount(RIGHT);
		float intvlR = GetInterval(RIGHT);
		float vr = countR / intvlR;

		// TODO: are anomalous velocities an issue with the Count4 peripheral (due to bad intervals)???
		// determine the velocity error and filter out anomalies
		float err = vl - vr;
		if (-MaxVelocityErr > err || err > MaxVelocityErr)
		{
			return;
		}

		// calculate the proportional component of the power adjustment
		float P = Kp * err;
printf("Velocity: LEFT: %u ticks / %f sec = %f t/s  RIGHT: %u ticks / %f sec = %f t/s   err = %f, P = %f\n", countL, intvlL, vl, countR, intvlR, vr, err, P);

		// TODO: I and D components must be calculated per-motor
		// for now calculate the power adjustment solely based on the proportional component
#if 1//def POWER_ADJ
		float powerL = GetPower(LEFT);
		float powerR = GetPower(RIGHT);
		float adjL = (P/2) * powerL;
		float adjR = (P/2) * powerR;
		float newPwrL = powerL - adjL;
		float newPwrR = powerR - adjR;
printf("Power: LEFT: %f - %f = %f  RIGHT: %f - %f = %f \n", powerL, adjL, newPwrL, powerR, adjL, newPwrL);
		SetPower(newPwrL, newPwrR);
#endif
    }
#else
    // simple balancing of the motor speeds
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


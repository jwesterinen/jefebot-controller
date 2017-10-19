/*
 * peripherals.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: jeff
 */

#ifndef DO_NOT_USE_RANDOM_VALUES
#include <unistd.h>
#endif
#include <cstdio>
#include <cassert>
#include <dp_events.h>
#include <dp_peripherals.h>
#include "peripherals.h"

UserInterface::UserInterface(DP::EventContext& evtCtx) : DP::BB4IO(evtCtx)
{
	evtCtx.Register(this);
	StartDataStream();
}

Locomotive::Locomotive(DP::EventContext& evtCtx, float _defaultSpeed) :
	DP::COUNT4(evtCtx, COUNT4_IDX), DP::DC2(evtCtx, DC2_IDX), direction(STOP), defaultSpeed(_defaultSpeed)
{
	// sanity check for default speed
	if (MinSpeed > defaultSpeed || defaultSpeed > MaxSpeed)
	{
		throw DP::FrameworkException("Locomotive speed", ERR_PARAMS);
	}

	// initialize the continuous tick counters
	ticks[0] = ticks[1] = 0;

	// register and configure the DP Count4 peripheral
	evtCtx.Register(this);
	SetUpdateRate(Count4Period);
	SetEdges(BOTH_EDGES, BOTH_EDGES, DISABLE_EDGE, DISABLE_EDGE);
	StartDataStream();

	// configure the DP DC2 peripheral:
	//   - stopped condition, i.e. modes = BREAK, powers = default
	//   - watchdog timeout
	Stop();
	SetWatchdog(WatchdogTimeout);
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
	    if (powers[LEFT] != powerL)
	    {
	        SetPower0(powers[LEFT] = powerL);
	    }
	    if (powers[RIGHT] != powerR)
	    {
	        SetPower1(powers[RIGHT] = powerR);
	    }
    }
}

void Locomotive::Stop()
{
    direction = STOP;
    SetMode(BREAK, BREAK);
    SetPower(defaultSpeed, defaultSpeed);
}

// move indefinitely if distance is 0
void Locomotive::MoveForward(unsigned distanceInCm)
{
    direction = MOVE_FORWARD;
    SetMode(FORWARD, FORWARD);
    if (distanceInCm > 0)
    {
    	MoveDistance(distanceInCm);
    	Stop();
    }
}

// move indefinitely if distance is 0
void Locomotive::MoveReverse(unsigned distanceInCm)
{
    direction = MOVE_REVERSE;
    SetMode(REVERSE, REVERSE);
    if (distanceInCm > 0)
    {
    	MoveDistance(distanceInCm);
    	Stop();
    }
}

void Locomotive::MoveDistance(unsigned distanceInCm)
{
#ifdef DO_NOT_USE_RANDOM_VALUES
	if (distanceInCm > 0)
	{
		volatile unsigned ticks;
		volatile unsigned beginTicks = GetTicks(RIGHT);
		do
		{
			ticks = GetTicks(RIGHT);
			printf("begin ticks = %d, ticks = %d\n", beginTicks, ticks);
		}
		while (TicksToCm(ticks - beginTicks) < distanceInCm);
	}
#else
	usleep(distanceInCm);
#endif
}

// spin indefinitely if distance is 0
void Locomotive::SpinCW(float angleInRadians)
{
    direction = SPIN_CW;
    SetMode(FORWARD, REVERSE);
    if (angleInRadians > 0)
    {
    	MoveAngle(angleInRadians);
    	Stop();
    }
}

// spin indefinitely if distance is 0
void Locomotive::SpinCCW(float angleInRadians)
{
    direction = SPIN_CCW;
    SetMode(REVERSE, FORWARD);
    if (angleInRadians > 0)
    {
    	MoveAngle(angleInRadians);
    	Stop();
    }
}

void Locomotive::MoveAngle(float angleInRadians)
{
#ifdef DO_NOT_USE_RANDOM_VALUES
	if (angleInRadians > 0.0)
	{
		volatile unsigned ticks;
		volatile unsigned beginTicks = GetTicks(RIGHT);
		do
		{
			ticks = GetTicks(RIGHT);
		}
		while (TicksToRadians(ticks - beginTicks) < angleInRadians);
	}
#else
	usleep(angleInRadians);
#endif
}

#define TICKS_PER_CM 1.9

bool Locomotive::HasMovedDistance(float distanceInCm)
{
	static unsigned beginTicks = (unsigned)-1;
	unsigned targetTicks = (unsigned)(distanceInCm * TICKS_PER_CM);

	if (beginTicks == (unsigned)-1)
	{
		beginTicks = GetTicks(RIGHT);
	}
printf("begin ticks = %d, ticks = %d\n", beginTicks, GetTicks(RIGHT));

	return ((GetTicks(RIGHT) - beginTicks) >= targetTicks);
}

#define TICKS_PER_RADIAN 14

bool Locomotive::HasTurnedAngle(float angleInRadians)
{
	unsigned targetTicks = angleInRadians * TICKS_PER_RADIAN;
	static unsigned beginTicks = (unsigned)-1;

	if (beginTicks == (unsigned)-1)
	{
		beginTicks = GetTicks(RIGHT);
	}
printf("target ticks = %d, begin ticks = %d, ticks = %d\n", targetTicks, beginTicks, GetTicks(RIGHT));

	return (GetTicks(RIGHT) - beginTicks >= targetTicks);
}

#define P_LOOP
void Locomotive::Handler()
{
	// call the counter's handler to get the current values
	DP::COUNT4::Handler();

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
//printf("Power: LEFT: %f - %f = %f  RIGHT: %f - %f = %f \n", powerL, adjL, newPwrL, powerR, adjL, newPwrL);
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

SinglePingRangeSensor::SinglePingRangeSensor(DP::EventContext& evtCtx, int _innerLimit, int _outerLimit) :
	DP::PING4(evtCtx, PING4_IDX), innerLimit(_innerLimit), outerLimit(_outerLimit)
{
	if (
		MinRange > innerLimit || innerLimit > MaxRange ||
		MinRange > outerLimit || outerLimit > MaxRange
	)
    {
    	throw DP::FrameworkException("SinglePingRangeSensor", ERR_PARAMS);
    }

	evtCtx.Register(this);
	Enable(SENSOR_0);
	StartDataStream();
}

// TODO: change class name to the specific brand/type of sensor
EdgeDetector::EdgeDetector(DP::EventContext& evtCtx, unsigned nominalEdgeLimit) : DP::ADC812(evtCtx, ADC812_IDX)
{
	if (MinEdgeRange > nominalEdgeLimit || nominalEdgeLimit > MaxEdgeRange)
    {
    	throw DP::FrameworkException("SinglePingRangeSensor", ERR_PARAMS);
    }
	for (int i = 0; i < 3; ++i)
	{
		edgeLimits[i] = nominalEdgeLimit;
	}

	evtCtx.Register(this);
	Config(Period, NO_PAIRS);
	StartDataStream();
}


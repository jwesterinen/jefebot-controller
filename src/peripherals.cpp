/*
 * peripherals.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: jeff
 */

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
	ClearTicks();

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

void Locomotive::ClearTicks()
{
	ticks[0] = ticks[1] = 0;

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

void Locomotive::MoveForward()
{
    direction = MOVE_FORWARD;
    SetMode(FORWARD, FORWARD);
}

void Locomotive::MoveReverse()
{
    direction = MOVE_REVERSE;
    SetMode(REVERSE, REVERSE);
}

void Locomotive::SpinCW()
{
    direction = SPIN_CW;
    SetMode(FORWARD, REVERSE);
}

void Locomotive::SpinCCW()
{
    direction = SPIN_CCW;
    SetMode(REVERSE, FORWARD);
}

bool Locomotive::HasMovedDistance(unsigned distanceInCm, unsigned* pCurDistance)
{
	static bool isMoving = false;
	static int beginTicks = 0;
	int targetTicks = distanceInCm * TicksPerCM;
	int ticks = (direction == MOVE_FORWARD) ? GetTicks(RIGHT) : -GetTicks(RIGHT);

	if (!isMoving)
	{
		beginTicks = ticks;
		isMoving = true;
	}

	// set the current distance
	if (pCurDistance)
	{
		*pCurDistance = ticks / TicksPerCM;
	}

	// debug print
	//printf("target ticks = %d, begin ticks = %d, ticks = %d\n", targetTicks, beginTicks, ticks);

	// return true if the distance has been met and cancel a distance measurement
	if (ticks - beginTicks >= targetTicks)
	{
		isMoving = false;
		return true;
	}

	return false;
}

bool Locomotive::HasTurnedAngle(float angleInRadians, float* pCurAngle)
{
	static bool isTurning = false;
	static int beginTicks = 0;
	int targetTicks = angleInRadians * TicksPerRadian;
	int ticks = GetTicks((direction == SPIN_CW) ? LEFT : RIGHT);

	// establish the beginning tick count if necessary
	if (!isTurning)
	{
		beginTicks = ticks;
		isTurning = true;
	}

	// set the current angle
	if (pCurAngle)
	{
		*pCurAngle = ticks / TicksPerRadian;
	}

	// debug print
	//printf("target ticks = %d, begin ticks = %d, ticks = %d\n", targetTicks, beginTicks, ticks);

	// return true if the angle has been met and cancel an angle measurement
	if (ticks - beginTicks >= targetTicks)
	{
		isTurning = false;
		return true;
	}

	return false;
}

#define P_LOOP
void Locomotive::Handler()
{
	// call the counter's handler to get the current values
	DP::COUNT4::Handler();

    // accumulate the ticks
    ticks[LEFT] += (GetMode(LEFT) == FORWARD) ? GetCount(LEFT) : -GetCount(LEFT);
    ticks[RIGHT] += (GetMode(RIGHT) == FORWARD) ? GetCount(RIGHT) : -GetCount(RIGHT);

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
		// debug pring
		//printf("Velocity: LEFT: %u ticks / %f sec = %f t/s  RIGHT: %u ticks / %f sec = %f t/s   err = %f, P = %f\n", countL, intvlL, vl, countR, intvlR, vr, err, P);

		// TODO: I and D components must be calculated per-motor
		// for now calculate the power adjustment solely based on the proportional component
		float powerL = GetPower(LEFT);
		float powerR = GetPower(RIGHT);
		float adjL = (P/2) * powerL;
		float adjR = (P/2) * powerR;
		float newPwrL = powerL - adjL;
		float newPwrR = powerR - adjR;
		// debug print
		//printf("Power: LEFT: %f - %f = %f  RIGHT: %f - %f = %f \n", powerL, adjL, newPwrL, powerR, adjL, newPwrL);
		SetPower(newPwrL, newPwrR);
    }
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

bool EdgeDetector::AtAnyEdge(enum EDGE_SENSORS* pEdge)
{
	if (AtEdge(LEFT))
	{
		if (pEdge)
		{
			*pEdge = LEFT;
		}
		return true;
	}
	else if (AtEdge(FRONT))
	{
		if (pEdge)
		{
			*pEdge = FRONT;
		}
		return true;
	}
	else if (AtEdge(RIGHT))
	{
		if (pEdge)
		{
			*pEdge = RIGHT;
		}
		return true;
	}

	return false;
}

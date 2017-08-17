/*
 * controller.c
 * 
 * Description:  This is the controller for jefebot.
 *   There are two modes that are selectable by the buttons and command line options:
 *     1. Roam: In this mode the jefebot moves forward until it detects an edge
 *        then avoids it and continues to move forward.
 *     2. GoToObject: In this mode the jefebot spins until it detects an object
 *        then it moves towards the object and stops when it arrives.
 */

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include "controller.h"

#define BETTER_AVOIDANCE

Controller::Controller(Context& ctx, bool _isVerbose) :
	Callback(Period),
	ui(ctx.ui), locomotive(ctx.locomotive), edgeDetector(ctx.edgeDetector), rangeSensor(ctx.rangeSensor),
	isVerbose(_isVerbose)
{
	// seed the random number gen
	srand((unsigned)time(&t));
}

#if 0
// turn right by a number of radians
void Controller::TurnRight(float radians)
{
	locomotive.SpinCW();
    locomotive.MoveAngle(radians);
    locomotive.Stop();
}

// turn left by a number of radians
void Controller::TurnLeft(float radians)
{
	locomotive.SpinCCW();
	locomotive.MoveAngle(radians);
    locomotive.Stop();
}
#endif

// move backwards by a number of CM
void Controller::BackUp(unsigned distance)
{
	locomotive.MoveReverse(distance);
}

RoamController::RoamController(Context& ctx, bool isVerbose) :
	Controller(ctx, isVerbose), state(ROAM)
{
	if (isVerbose) printf("changing state to ROAM\n");
	locomotive.Stop();
	locomotive.MoveForward(0);
	ui.Display(0x01);
}

void RoamController::Routine()
{
	switch (state)
	{
		case ROAM:
			if (edgeDetector.AtAnyEdge())
			{
				locomotive.Stop();
				state = AVOID_EDGE;
				if (isVerbose) printf("changing state to AVOID_EDGE\n");
			}
			else if (rangeSensor.AtObject())
			{
				if (isVerbose)
				{
					printf("object found at distance = %u\n", rangeSensor.GetDistance());
				}
				locomotive.Stop();
				state = AVOID_OBJECT;
				if (isVerbose) printf("changing state to AVOID_OBJECT\n");
			}
			break;

		case AVOID_EDGE:
			if (edgeDetector.AtEdge(EdgeDetector::LEFT))
			{
				locomotive.Stop();
				BackUp(300000);
				locomotive.Stop();
				locomotive.SpinCW(RANDOM_VALUE);
			}
			else if (edgeDetector.AtEdge(EdgeDetector::FRONT))
			{
// TODO: check for simultaneous left and right edges
#ifdef DETECT_BOTH_EDGES
				while (edgeDetector.AtEdge(EdgeDetector::LEFT) && edgeDetector.AtEdge(EdgeDetector::RIGHT))
				{
					BackUp(RANDOM_VALUE);
				}
#endif
				if (edgeDetector.AtEdge(EdgeDetector::LEFT))
				{
					locomotive.Stop();
					BackUp(300000);
					locomotive.Stop();
					locomotive.SpinCW(RANDOM_VALUE);
				}
				else
				{
					locomotive.Stop();
					BackUp(300000);
					locomotive.Stop();
					locomotive.SpinCCW(RANDOM_VALUE);
				}
			}
			else if (edgeDetector.AtEdge(EdgeDetector::RIGHT))
			{
				locomotive.Stop();
				BackUp(300000);
				locomotive.Stop();
				locomotive.SpinCCW(RANDOM_VALUE);
			}
			locomotive.MoveForward(0);
			state = ROAM;
			if (isVerbose) printf("changing state to ROAM\n");
			break;

		case AVOID_OBJECT:
// TODO: check for simultaneous left and right edges
#ifdef DETECT_BOTH_EDGES
			while (edgeDetector.AtEdge(EdgeDetector::LEFT) && edgeDetector.AtEdge(EdgeDetector::RIGHT))
			{
				BackUp(RANDOM_VALUE);
			}
#endif
			(edgeDetector.AtEdge(EdgeDetector::LEFT)) ? locomotive.SpinCW(RANDOM_VALUE) : locomotive.SpinCCW(RANDOM_VALUE);
			locomotive.MoveForward(0);
			state = ROAM;
			if (isVerbose) printf("changing state to ROAM\n");
			break;

		default:
			assert(false);
	}
}

GotoObjectController::GotoObjectController(Context& ctx, bool isVerbose) :
		Controller(ctx, isVerbose), state(FIND_OBJECT)
{
	if (isVerbose) printf("changing state to FIND_OBJECT\n");
	locomotive.Stop();
	locomotive.SpinCW(0);
	ui.Display(0x02);
}

void GotoObjectController::Routine()
{
	unsigned distance;
	static unsigned tickCount = 0, targetCount = 0;

	switch (state)
	{
		case FIND_OBJECT:
			if (rangeSensor.DetectObject(&distance))
			{
				// get the tick count
				tickCount = locomotive.GetTicks(Locomotive::LEFT);
				targetCount = tickCount;
				state = MEASURE_OBJECT;
				if (isVerbose)
				{
					printf("object found at distance %d\n", distance);
					printf("TickCount = %d\n", tickCount);
					printf("changing state to MEASURE_OBJECT\n");
				}
			}
			break;

		case MEASURE_OBJECT:
			// continue until the object is undetected
			if (!rangeSensor.DetectObject(&distance))
			{
				int tickDelta;

				locomotive.Stop();
				tickCount = locomotive.GetTicks(Locomotive::LEFT);
				if (isVerbose)
				{
					printf("TickCount = %d\n", tickCount);
				}
				tickDelta = ((tickCount - targetCount) / 2);
				targetCount += tickDelta - TRIM;
				if (isVerbose)
				{
					printf("TargetCount = %d\n", targetCount);
				}
				// TODO: change to a specific amount of angle to spin to eliminate the adjust position state
				locomotive.SpinCCW(0);
				state = ADJUST_POSITION;
				if (isVerbose) printf("changing state to ADJUST_POSITION\n");
			}
			break;

		case ADJUST_POSITION:
			// continue until the tick count is less than the target
			tickCount = locomotive.GetTicks(Locomotive::LEFT);
			if (tickCount <= targetCount)
			{
				// the middle of the object has bee found so go to it
				locomotive.Stop();
				if (isVerbose)
				{
					printf("TickCount = %d\n", tickCount);
				}
				locomotive.MoveForward(0);
				state = GOTO_OBJECT;
				if (isVerbose) printf("changing state to GOTO_OBJECT\n");
			}
			break;

		case GOTO_OBJECT:
			// TODO: replace this with a PID controller algorithm
			if (!rangeSensor.DetectObject(&distance))
			{
				locomotive.SpinCW(0);
				state = FIND_OBJECT;
				if (isVerbose) printf("changing state to FIND_OBJECT\n");
			}
			else if (rangeSensor.AtObject())
			{
				state = PUSH_OBJECT;
				if (isVerbose) printf("changing state to FIND_OBJECT\n");
			}
			else if (edgeDetector.AtAnyEdge())
			{
				locomotive.Stop();
				state = AVOID_EDGE;
				if (isVerbose) printf("changing state to AVOID_EDGE\n");
			}
			break;

		case PUSH_OBJECT:
			if (edgeDetector.AtAnyEdge())
			{
				printf("objective achieved...\n");
				Shutdown("", 0);
			}
			break;

		case AVOID_EDGE:
// TODO: need better avoidance
#ifdef BETTER_AVOIDANCE
			BackUp(300);
#endif
			locomotive.SpinCW(0);
			state = FIND_OBJECT;
			if (isVerbose) printf("changing state to FIND_OBJECT\n");
			break;

		default:
			assert(false);
	}
}


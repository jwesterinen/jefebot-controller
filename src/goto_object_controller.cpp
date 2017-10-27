/*
 * goto_object_controller.c
 * 
 * Description:  This is the controller for jefebot.
 *   There are two modes that are selectable by the buttons and command line options:
 *     1. Roam: In this mode the jefebot moves forward until it detects an edge
 *        then avoids it and continues to move forward.
 *     2. GoToObject: In this mode the jefebot spins until it detects an object
 *        then it moves towards the object and stops when it arrives.
 */

#include "goto_object_controller.h"

//#define TRIM 0
//#define TRIM 1
#define TRIM 2
//#define TRIM 3

GotoObjectController::GotoObjectController(Context& ctx, bool isVerbose) :
		Controller(ctx, isVerbose), state(ESTABLISH_RANGE), objDistance(-1)
{
	if (isVerbose) printf("changing state to ESTABLISH_RANGE\n");
	locomotive.SpinCW();
	ui.Display(0x02);
}

void GotoObjectController::Routine()
{
	unsigned distance;
	static int tickCount = 0, targetCount = 0;

	switch (state)
	{
		case ESTABLISH_RANGE:
			if (!locomotive.HasTurnedAngle(6.5))
			{
				if (rangeSensor.DetectObject(0, &distance))
				{
					if (distance < objDistance)
					{
						objDistance = distance;
					}
				}
			}
			else
			{
				if (isVerbose)
				{
					printf("object found at distance %d\n", objDistance);
					printf("changing state to FIND_OBJECT\n");
				}
				objDistance += 10;
				locomotive.Stop();
				locomotive.ClearTicks();
				locomotive.SpinCW();
				state = FIND_OBJECT;
			}
			break;

		case FIND_OBJECT:
			if (rangeSensor.DetectObject(objDistance, &distance))
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
			if (!rangeSensor.DetectObject(objDistance, &distance))
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
				locomotive.SpinCCW();
				state = ADJUST_POSITION;
				if (isVerbose)
				{
					printf("changing state to ADJUST_POSITION\n");
				}
			}
			break;

		case ADJUST_POSITION:
			// continue until the tick count is less than the target
			tickCount = locomotive.GetTicks(Locomotive::LEFT);
			if (tickCount <= targetCount)
			{
				// the middle of the object has been found so go to it
				locomotive.Stop();
				if (isVerbose)
				{
					printf("TickCount = %d\n", tickCount);
				}
				locomotive.MoveForward();
				state = GOTO_OBJECT;
				if (isVerbose) printf("changing state to GOTO_OBJECT\n");
			}
			break;

		case GOTO_OBJECT:

			if (rangeSensor.AtObject())
			{
				state = PUSH_OBJECT;
				if (isVerbose)
				{
					printf("object reached at distance %d\n", rangeSensor.GetDistance());
					printf("changing state to PUSH_OBJECT\n");
				}
			}
			else if (!rangeSensor.DetectObject(objDistance, &distance))
			{
				locomotive.SpinCW();
				state = FIND_OBJECT;
				if (isVerbose)
				{
					printf("object lost at distance %d\n", distance);
					printf("changing state to FIND_OBJECT\n");
				}
			}
			else if (edgeDetector.AtAnyEdge())
			{
				locomotive.Stop();
				state = AVOID_EDGE;
				if (isVerbose)
				{
					printf("edge found:\n");
					printf("  left sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::LEFT));
					printf("  front sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::FRONT));
					printf("  right sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::RIGHT));
					printf("changing state to AVOID_EDGE\n");
				}
			}
			break;

		case PUSH_OBJECT:
			if (!rangeSensor.AtObject())
			{
				if (isVerbose)
				{
					printf("object lost at distance %d\n", rangeSensor.GetDistance());
				}
				Shutdown("objective achieved...\n", 0);
			}
			else if (edgeDetector.AtAnyEdge())
			{
				locomotive.Stop();
				Shutdown("edge detected", 0);
				if (isVerbose)
				{
					printf("edge found:\n");
					printf("  left sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::LEFT));
					printf("  front sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::FRONT));
					printf("  right sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::RIGHT));
				}
			}
			break;

		case AVOID_EDGE:
// TODO: need better avoidance
#ifdef BETTER_AVOIDANCE
			BackUp(300);
#endif
			locomotive.SpinCW();
			state = FIND_OBJECT;
			if (isVerbose) printf("changing state to FIND_OBJECT\n");
			break;

		default:
			assert(false);
	}
}


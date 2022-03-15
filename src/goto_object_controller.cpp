/*
 *  goto_object_controller.cpp
 * 
 *  Description:  This is the "go to object" controller for jefebot.  In this mode, jefebot
 *  will find an object on the table, go to it, and push it off.  The algorithm is as follows:
 *      1. Spin one complete revolution CW to find the closest object within bounds and save the distance.
 *      2. Spin CW until the object is first detected again and save this "found" tick count.  
 *      3. Continue to spin CW until the object is lost and save that "lost" tick count. Calculate 
 *         the tick count to spin CCW to point to the theoretical middle of the object by spliting
 *         the difference of the "lost" and "found" tick counts.
 *      4. Spin CCW by the amount calculated in step 3 to point to the middle of the object.
 *      5. Move forward to the object all the time making sure the object doesn't get lost or 
 *         encounter an edge, due to the bot's drifting off course.  If the object is lost,
 *         go back to step 2; if an edge is detected, just stop.
 *      6. Continue to move forward to push the object off the table, making sure no edges are
 *         encountered.  If the front edge is detected, the object is presumably pushed off the
 *         table, but if any other edges are encountered, just stop.
 *      7. Immediately move back a few cenimeters to prevent the bot from itself falling off.
 *
 *  The controller is implemented as a state machine with the first 7 states corresponding
 *  to the steps of the algorithm described above.  There are 2 extra states, one that is 
 *  entered when an edge is encountered, and a final, completion state.
 *
 *  The Routine() function is registered in the main program as a periodic event handler, and
 *  is therefore continually called at a rate specified during its registration.
*/

#include "stdlib.h"
#include "goto_object_controller.h"

//#define TRIM 0
#define TRIM 1
//#define TRIM 2
//#define TRIM 3

GotoObjectController::GotoObjectController(Context& ctx, bool isVerbose) :
		Controller(ctx, isVerbose), state(ESTABLISH_RANGE), objDistance(-1)
{
	if (isVerbose) printf("changing state to ESTABLISH_RANGE...\n");
	angleToTurn = 2*PI;
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
			// start the process by spinning 2pi radians CW to find the range of the closest object
			if (!locomotive.HasTurnedAngle(angleToTurn))
			{
				if (rangeSensor.DetectObject(0, &distance))
				{
				    // keep updating the range of the closest object within bounds as the bot spins
					if (distance < objDistance)
					{
						objDistance = distance;
					}
				}
			}
			else
			{
			    // once the range has been established go on to find the object
				if (isVerbose)
				{
					printf("object found at distance %d\n", objDistance);
					printf("changing state to FIND_OBJECT...\n");
				}

				// adjust the range a little farther, clear the heading (ticks) then go on to find the object
				objDistance += 10;
				locomotive.Stop();
				locomotive.ClearTicks();
				locomotive.SpinCW();
				state = FIND_OBJECT;
			}
			break;

		case FIND_OBJECT:
		    // spin CW until the object is first detected in the established range
			if (rangeSensor.DetectObject(objDistance, &distance))
			{
				// get the tick count when the object is first detected
				tickCount = locomotive.GetTicks(Locomotive::LEFT);
				targetCount = tickCount;
				state = MEASURE_OBJECT;
				if (isVerbose)
				{
					printf("object found at distance %d\n", distance);
					printf("TickCount = %d\n", tickCount);
					printf("changing state to MEASURE_OBJECT...\n");
				}
			}
			break;

		case MEASURE_OBJECT:
			// continue spinning until the object is undetected
			if (!rangeSensor.DetectObject(objDistance, &distance))
			{
				int tickDelta;

				locomotive.Stop();
				
				// get the tick count of when the object is first undetected
				tickCount = locomotive.GetTicks(Locomotive::LEFT);
				if (isVerbose)
				{
					printf("TickCount = %d\n", tickCount);
				}
				
				// calculate the amount to spin CCW to point to the middle of the object
				tickDelta = ((tickCount - targetCount) / 2);
				targetCount += tickDelta - TRIM;
				if (isVerbose)
				{
					printf("TargetCount = %d\n", targetCount);
				}
				// TODO: change to a specific amount of angle to spin to eliminate the adjust position state
				//       -- this will require much more accurate odometry
				locomotive.SpinCCW();
				state = ADJUST_POSITION;
				if (isVerbose)
				{
					printf("changing state to ADJUST_POSITION...\n");
				}
			}
			break;

		case ADJUST_POSITION:
			// spin CCW by the amount calculated to point to the theoretical middle of the object
			tickCount = locomotive.GetTicks(Locomotive::LEFT);
			if (tickCount <= targetCount)
			{
				// the middle of the object has been found so move forward to it
				locomotive.Stop();
				if (isVerbose)
				{
					printf("TickCount = %d\n", tickCount);
				}
				locomotive.MoveForward();
				if (isVerbose) printf("changing state to GOTO_OBJECT...\n");
				state = GOTO_OBJECT;
			}
			break;

		case GOTO_OBJECT:
			if (rangeSensor.AtObject())
			{
			    // when the bot is at the object go on to push it
				state = PUSH_OBJECT;
				if (isVerbose)
				{
					printf("object reached at distance %d\n", rangeSensor.GetDistance());
					printf("changing state to PUSH_OBJECT...\n");
				}
			}
			else if (!rangeSensor.DetectObject(objDistance, &distance))
			{
			    // the object was lost so try to find it again
				locomotive.SpinCW();
				state = FIND_OBJECT;
				if (isVerbose)
				{
					printf("object lost at distance %d\n", distance);
					printf("changing state to FIND_OBJECT...\n");
				}
			}
			else if (edgeDetector.AtAnyEdge())
			{
			    // need to avoid any edge at this point
		        if (isVerbose) printf("changing state to AVOID_EDGE...\n");
		        state = AVOID_EDGE;
			}
			break;

		case PUSH_OBJECT:
			if (edgeDetector.AtAnyEdge(&edge))
			{
				switch (edge)
				{
					case EdgeDetector::FRONT:
					    // when the front edge is detected, the object has been pushed off the table, so stop the bot and immediately back up to prevent from falling off with the object
						locomotive.Stop();
				        distanceToMove = 6;
				        locomotive.MoveReverse();
				        if (isVerbose) printf("changing state to PREVENT_FALLING...\n");
				        state = PREVENT_FALLING;
						break;
					default:
					    // any other edge is a problem so need to avoid it
				        if (isVerbose) printf("changing state to AVOID_EDGE...\n");
				        state = AVOID_EDGE;
						break;
				}
			}
			break;

		case AVOID_EDGE:
			// simply stop to avoid the edge then shutdown			    
			locomotive.Stop();
		    system("/home/jefebot/controller/sounds/play_wawa.bat");
			Shutdown("...edge detected\n", 0);
			break;

		case PREVENT_FALLING:
			if (locomotive.HasMovedDistance(distanceToMove))
			{
			    // the bot has moved back to avoid falling off with the object so complete the objective
				locomotive.Stop();
				state = COMPLETE;
			}
			break;

		case COMPLETE:
		    system("/home/jefebot/controller/sounds/play_woohoo.bat");
			Shutdown("...objective achieved\n", 0);
			break;

		default:
			assert(false);
	}
}


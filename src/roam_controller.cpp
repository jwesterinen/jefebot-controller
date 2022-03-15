/*
 *  roam_controller.cpp
 * 
 *  Description:  This is the "roam" controller for jefebot.  In this mode, jefebot
 *  will traverse a table without falling off.  The algorithm is as follows:
 *      1. Move forward until an edge or an object is detected then stop.
 *      2. Backup 3cm.
 *      3. If the left edge was detected, a request is made to turn .8 radians clockwise.  If
 *         the right edge was detected, a request is made to turn .8 radians counter clockwise.
 *         If the front edge was detected, a request is made to turn 1.6 radians counter clockwise.
 *      4. Make the requested turn from state 3, then move forward and return to step 1.
 *
 *  The controller is implemented as a state machine with 4 states corresponding
 *  to the steps of the algorithm described above.  There is no completion state; roaming
 *  will continue until the right-most button on jefebot is pressed.
 *
 *  The Routine() function is registered in the main program as a periodic event handler, and
 *  is therefore continually called at a rate specified during its registration.
 */

#include "roam_controller.h"

RoamController::RoamController(Context& ctx, bool isVerbose) :
	Controller(ctx, isVerbose), state(ROAM)

{
	if (isVerbose) printf("changing state to ROAM\n");
	ui.Display(0x01);
	locomotive.MoveForward();
}

void RoamController::Routine()
{
	switch (state)
	{
		case ROAM:
			if (edgeDetector.AtAnyEdge(&edge))
			{
				if (isVerbose)
				{
					printf("edge %d found:\n", edge);
					printf("  left sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::LEFT));
					printf("  front sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::FRONT));
					printf("  right sensor value = %d\n", edgeDetector.GetEdgeSensorValue(EdgeDetector::RIGHT));
				}
				if (isVerbose) printf("changing state to BACKUP\n");
				locomotive.Stop();
				distanceToMove = 3;
				locomotive.MoveReverse();
				state = BACKUP;
			}
			else if (rangeSensor.AtObject())
			{
				edge = EdgeDetector::FRONT;
				locomotive.Stop();
				distanceToMove = 3;
				locomotive.MoveReverse();
				if (isVerbose) printf("changing state to BACKUP\n");
				state = BACKUP;
			}
			break;

		case BACKUP:
			if (locomotive.HasMovedDistance(distanceToMove))
			{
				if (isVerbose) printf("changing state to AVOID_EDGE\n");
				state = AVOID_EDGE;
			}
			break;

		case AVOID_EDGE:
			locomotive.Stop();
			switch (edge)
			{
				case EdgeDetector::LEFT:
					angleToTurn = 0.8;
					locomotive.SpinCW();
					break;
				case EdgeDetector::RIGHT:
					angleToTurn = 0.8;
					locomotive.SpinCCW();
					break;
				case EdgeDetector::FRONT:
					angleToTurn = 1.6;
					locomotive.SpinCCW();
					break;
				default:
					angleToTurn = 1.6;
					locomotive.SpinCCW();
					break;
			}
			if (isVerbose) printf("changing state to TURN\n");
			state = TURN;
			break;

		case TURN:
			if (locomotive.HasTurnedAngle(angleToTurn))
			{
				locomotive.Stop();
				locomotive.MoveForward();
				if (isVerbose) printf("changing state to ROAM\n");
				state = ROAM;
			}
			break;


		default:
			assert(false);
	}
}

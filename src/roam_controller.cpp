/*
 * roam_controller.c
 * 
 * Description:  This is the controller for jefebot.
 *   There are two modes that are selectable by the buttons and command line options:
 *     1. Roam: In this mode the jefebot moves forward until it detects an edge
 *        then avoids it and continues to move forward.
 *     2. GoToObject: In this mode the jefebot spins until it detects an object
 *        then it moves towards the object and stops when it arrives.
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

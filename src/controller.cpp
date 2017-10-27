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

#include "controller.h"

#define BETTER_AVOIDANCE

Controller::Controller(Context& ctx, bool _isVerbose) :
	Callback(Period),
	ui(ctx.ui), locomotive(ctx.locomotive), edgeDetector(ctx.edgeDetector), rangeSensor(ctx.rangeSensor),
	isVerbose(_isVerbose), 	distanceToMove(0), angleToMove(0)

{
#ifndef DO_NOT_USE_RANDOM_VALUES
	// seed the random number gen
	srand((unsigned)time(&t));
#endif
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

// move backwards by a number of CM
void Controller::BackUp(unsigned distance)
{
	locomotive.MoveReverse(distance);
}
#endif

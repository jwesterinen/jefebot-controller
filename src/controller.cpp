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

Controller::Controller(Context& ctx, bool _isVerbose) :
	Callback(Period),
	ui(ctx.ui), locomotive(ctx.locomotive), edgeDetector(ctx.edgeDetector), rangeSensor(ctx.rangeSensor),
	isVerbose(_isVerbose), edge(EdgeDetector::LEFT), 	distanceToMove(0), angleToTurn(0.0)

{
}

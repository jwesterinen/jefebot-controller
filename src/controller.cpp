/*
 * controller.cpp
 * 
 * Description:  This is the implementation of the Controller class.
 *
 */

#include "controller.h"

Controller::Controller(Context& ctx, bool _isVerbose) :
	Callback(Period),
	ui(ctx.ui), locomotive(ctx.locomotive), edgeDetector(ctx.edgeDetector), rangeSensor(ctx.rangeSensor),
	isVerbose(_isVerbose), edge(EdgeDetector::LEFT), 	distanceToMove(0), angleToTurn(0.0)

{
}

/*
 *  controller.h
 * 
 *  Description:  This is the base class for any behavior control program for jefebot.
 *      All controllers have the following properties that must be defined:
 *          - Callback:  the event handler for the controller
 *          - ui:                the physical user interface objecgt used to control jefebot
 *          - locomotive:        the locomotive object
 *          - edgeDetector:      the edge detector object
 *          - rangeSensor:       the range sensor object
 *          - ifVerbose:         degree of verbosity flag
 *          - edge:              ???
 *          - distanceToMove:    distance variable
 *          - angleToTurn:       angle variable
 */

#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include "peripherals.h"
#define PI 3.14

class Controller : public DP::Callback
{
private:
    static const unsigned Period = 50;

protected:
    UserInterface& ui;
	Locomotive& locomotive;
	EdgeDetector& edgeDetector;
	SinglePingRangeSensor& rangeSensor;
	bool isVerbose;
	enum EdgeDetector::EDGE_SENSORS edge;
	int distanceToMove;
	float angleToTurn;

public:
	struct Context
	{
	    UserInterface& ui;
		Locomotive& locomotive;
		EdgeDetector& edgeDetector;
		SinglePingRangeSensor& rangeSensor;
		Context(
			UserInterface& _ui,
			Locomotive& _locomotive,
			EdgeDetector& _edgeDetector,
			SinglePingRangeSensor& _rangeSensor
		) : ui(_ui), locomotive(_locomotive), edgeDetector(_edgeDetector), rangeSensor(_rangeSensor)
		{}
	};

	Controller(Context& ctx, bool _isVerbose);
	virtual ~Controller()
	{}
};

#endif /* INCLUDE_CONTROLLER_H_ */

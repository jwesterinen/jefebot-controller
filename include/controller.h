#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include "peripherals.h"
#ifndef DO_NOT_USE_RANDOM_VALUES
#include <cstdlib>
#define RANDOM_VALUE ((rand() % 8) * 100000)
#endif
#define PI 3.14

class Controller : public DP::Callback
{
private:
    time_t t;
    static const unsigned Period = 50;

protected:
    UserInterface& ui;
	Locomotive& locomotive;
	EdgeDetector& edgeDetector;
	SinglePingRangeSensor& rangeSensor;
	bool isVerbose;
	int distanceToMove;
	float angleToMove;

#if 0
	virtual void TurnRight(float radians);
	virtual void TurnLeft(float radians);
	virtual void BackUp(unsigned distance);
#endif

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

#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include "peripherals.h"

// TODO: make turning actually use odometry to figure out radians
#define RANDOM_VALUE ((rand() % 8) * 100000)

#define TRIM 3

class Controller : public Callback
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

class RoamController : public Controller
{
private:
	enum STATE {ROAM, AVOID_EDGE, AVOID_OBJECT} state;

protected:
	void Routine();

public:
	RoamController(Context& ctx, bool isVerbose);
	~RoamController()
	{}
};

class GotoObjectController : public Controller
{
private:
	enum STATE {FIND_OBJECT, MEASURE_OBJECT, ADJUST_POSITION, GOTO_OBJECT, PUSH_OBJECT, AVOID_EDGE} state;

protected:
	void Routine();

public:
	GotoObjectController(Context& ctx, bool isVerbose);
	~GotoObjectController()
	{}
};

#endif /* INCLUDE_CONTROLLER_H_ */

#ifndef INCLUDE_GOTO_OBJECT_CONTROLLER_H_
#define INCLUDE_GOTO_OBJECT_CONTROLLER_H_

#include "controller.h"

class GotoObjectController : public Controller
{
private:
	enum STATE {ESTABLISH_RANGE, FIND_OBJECT, ROTATE_TO_OBJECT, MEASURE_OBJECT, MOVE_TO_OBJECT, ADJUST_POSITION, GOTO_OBJECT, PUSH_OBJECT, AVOID_EDGE, PRE_COMPLETE, COMPLETE} state;
	unsigned objDistance;

protected:
	void Routine();

public:
	GotoObjectController(Context& ctx, bool isVerbose);
	~GotoObjectController()
	{}
};

#endif /* INCLUDE_GOTO_OBJECT_CONTROLLER_H_ */

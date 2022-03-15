/*
 *  goto_object_controller.h
 *
 *  Description: Class to implement a behavior control program for jefebot that performs
 *  the requirements of the HBRC Table Top Challenge level 2, e.g. find an object on a table,
 *  go to it, then push it off the table without itself falling off.  The details of how this 
 *  is implemented is in the file goto_object_controller.cpp.
 *
 *  Created on: Mar 2, 2017
 *      Author: jeff
 */

#ifndef INCLUDE_GOTO_OBJECT_CONTROLLER_H_
#define INCLUDE_GOTO_OBJECT_CONTROLLER_H_

#include "controller.h"

class GotoObjectController : public Controller
{
private:
	enum STATE {ESTABLISH_RANGE, FIND_OBJECT, ROTATE_TO_OBJECT, MEASURE_OBJECT, MOVE_TO_OBJECT, ADJUST_POSITION, GOTO_OBJECT, PUSH_OBJECT, AVOID_EDGE, PREVENT_FALLING, COMPLETE} state;
	unsigned objDistance;

protected:
	void Routine();

public:
	GotoObjectController(Context& ctx, bool isVerbose);
	~GotoObjectController()
	{}
};

#endif /* INCLUDE_GOTO_OBJECT_CONTROLLER_H_ */

/*
 *  roam_controller.h
 *
 *  Description: Class to implement a behavior control program for jefebot that performs
 *  the requirements of the HBRC Table Top Challenge level 1, e.g. traverse a table without
 *  falling off.  The details of how this is implemented is in the file roam_controller.cpp.
 *
 *  Created on: Mar 2, 2017
 *      Author: jeff
 */

#ifndef INCLUDE_ROAM_CONTROLLER_H_
#define INCLUDE_ROAM_CONTROLLER_H_

#include "controller.h"

class RoamController : public Controller
{
private:
	enum STATE {ROAM, BACKUP, AVOID_EDGE, TURN} state;

protected:
	void Routine();

public:
	RoamController(Context& ctx, bool isVerbose);
	~RoamController()
	{}
};

#endif /* INCLUDE_ROAM_CONTROLLER_H_ */

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

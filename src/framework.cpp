/*
 * Framework.cpp
 *
 *  Created on: Mar 7, 2017
 *      Author: jeff
 */

//#include <stdio.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <arpa/inet.h>
#include <sys/time.h>
#include "framework.h"

int Framework::cmdfd = -1;

Callback::Callback(unsigned _period) : period(_period)
{
	// get "now"
	struct timeval tv;
	gettimeofday(&tv, 0);

	// convert to timeout
	to = Framework::Tv2us(&tv) + (period * 1000);
	period = period * 1000;
}

bool Callback::HasExpired(unsigned long long now)
{
    return (to <= now);
}
void Callback::UpdateTimeout(unsigned long long now)
{
	to += period;
	if (to < now)
	{
		/* CPU hog made us miss a period? */
		to = now;
	}
}

unsigned long long Callback::GetNextTimeout(unsigned long long curTimeout)
{
	unsigned long long nextto = curTimeout;

    if ((nextto == (unsigned long long)-1) || (to < nextto))
    {
        nextto = to;
    }

    return nextto;
}

Framework::Framework() : maxFd(0)
{
	// init the global command FD
    Framework::cmdfd = OpenConnection();
}

/***************************************************************************
 * tv2us(): - convert a timeval struct to long long of microseconds.
 *
 * Input:        Pointer to a timer structure
 * Output:       long long
 * Effects:      No side effects
 ***************************************************************************/
unsigned long long Framework::Tv2us(struct timeval *ptv)
{
    return ((((unsigned long long)ptv->tv_sec) * 1000000) + ptv->tv_usec);
}

int Framework::OpenConnection()
{
    struct sockaddr_in skt;
    int fd;

    // open a connection to DPserver daemon to send commands
    int adrlen = sizeof(struct sockaddr_in);
    (void) memset((void *) &skt, 0, (size_t) adrlen);
    skt.sin_family = AF_INET;
    skt.sin_port = htons(8880);
    if ((inet_aton("127.0.0.1", &(skt.sin_addr)) == 0) ||
        ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) ||
        (connect(fd, (struct sockaddr *) &skt, adrlen) < 0))
    {
    	throw FrameworkException("Framework", ERR_CONNECT);
    }

    return fd;
}

// add a DP peripheral handler
void Framework::Register(SelectableSensor* sensor)
{
	// open a connection to the DP server and add it to the peripherals list
    sensor->dataFd = OpenConnection();
    maxFd = sensor->dataFd;
    selectableSensors.push_back(sensor);
}

// register a pre-defined PeriodicRoutine object that supplies its own FD
// and will have its Handler() function called
void Framework::Register(Callback* routine)
{
    periodicRoutines.push_back(routine);
}


/***************************************************************************
 * ExecRoutines(): - Scan the timer queue looking for expired timers.
 * Call the callbacks for any expired timers and either remove
 * them (ED_ONESHOT) or reschedule them (ED_PERIODIC).
 *   Output a NULL timeval pointer if there are no timer or a
 * pointer to a valid timeval struct if there are timers.
 *
 * Input:        none
 * Output:       pointer to a timeval struct
 * Effects:      none
 ***************************************************************************/
timeval* Framework::ExecRoutines()
{
    struct timeval tv;  				// timeval struct to hold "now"
    unsigned long long now;     		// "now" in milliseconds since Epoch
    unsigned long long nextTimeout;
    std::list<Callback*>::iterator it;

    // the following is the allocation for the timeout used in select()
    static struct timeval selectTimeout;

    // just return if no routines are registered
    if (periodicRoutines.empty())
    {
        return ((struct timeval*)0);
    }

    // get "now" in milliseconds since the Epoch
    if (gettimeofday(&tv, 0) < 0)
    {
        return ((struct timeval*)0);
    }
    now = Framework::Tv2us(&tv);

    // search for a routine with a timeout less than now
    for (it = periodicRoutines.begin(); it != periodicRoutines.end(); ++it)
    {
       // call the routine if its period has been exceeded
    	if ((*it)->HasExpired(now))
        {
			// call the routine and update its timeout
			(*it)->Routine();
        	(*it)->UpdateTimeout(now);
        }
    }

    // with all routines dispatched, set the timeout to be used in the next select call
    // note: this is null if there are no routines otherwise the select timeout is based
    // on the next routine's timeout value

    // Walk the timer array again to find the nearest timeout
    nextTimeout = -1;
    for (it = periodicRoutines.begin(); it != periodicRoutines.end(); ++it)
    {
        nextTimeout = (*it)->GetNextTimeout(nextTimeout);
    }
    if ((nextTimeout - now) < 0)
    {
    	// next timeout is in the past (CPU hog?)
        nextTimeout = now;
    }
    selectTimeout.tv_sec = (nextTimeout - now) / 1000000;
    selectTimeout.tv_usec = (suseconds_t) ((nextTimeout - now) % 1000000);

    return (&selectTimeout);
}

void Framework::MainEventLoop()
{
    fd_set readset;
    struct timeval *pSelectTimeout;
    std::list<SelectableSensor*>::iterator it;

    while (1)
    {
		// init the local fd sets from the global ones
		FD_ZERO(&readset);
		for (it = selectableSensors.begin(); it != selectableSensors.end(); ++it)
		{
			FD_SET((*it)->dataFd, &readset);
		}

		// dispatch all periodic routines as necessary
		pSelectTimeout = ExecRoutines();

		// wait for FD activity
		if (select(maxFd + 1, &readset, NULL, NULL, pSelectTimeout) < 0)
		{
			// select error -- bail out on all but EINTR
			if (errno != EINTR)
		    {
		    	throw FrameworkException("Framework", ERR_SELECT);
		    }
		}

		// dispatch all DP peripheral handlers as necessary
		for (it = selectableSensors.begin(); it != selectableSensors.end(); ++it)
		{
			if (FD_ISSET((*it)->dataFd, &readset))
			{
				(*it)->Handler();
			}
		}
    }
}

int main(int argc, char* argv[])
{
	Framework framework;

	try
	{
		// initialize the controller by calling the required init routine
		InitControlProgram(argc, argv, framework);

		// dispatch all DP peripheral handlers and periodic routines as necessary
		framework.MainEventLoop();

	} catch (FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

    // execution will never get here
    exit(EXIT_SUCCESS);
}


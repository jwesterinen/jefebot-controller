/*
 * Framework.h
 *
 *  Created on: Mar 7, 2017
 *      Author: jeff
 */

#ifndef INCLUDE_FRAMEWORK_H_
#define INCLUDE_FRAMEWORK_H_

#include <list>
#include <string>

// framework errors
#define ERR_NONE			 0
#define ERR_UNKNOWN			-1
#define ERR_INITIALIZATION	-1001
#define ERR_CONNECT			-1002
#define ERR_READ			-1003
#define ERR_WRITE			-1004
#define ERR_SELECT			-1005
#define ERR_PARAMS			-1006
#define ERR_RESPONSE		-1007
#define ERR_REGISTRATION	-1008

// framework exception
class FrameworkException : public std::exception
{
private:
	std::string msg;
	int error;

public:
	FrameworkException(const char* _msg, int _error) : msg(_msg), error(_error)
	{}
	~FrameworkException() throw()
	{}
	virtual const char* what() const throw()
	{
		return msg.c_str();
	}
	int Error()
	{
		return error;
	}
};

// periodic routine context
class Callback
{
friend class Framework;

private:
	unsigned long long to;		// uSec since the epoch of Jan 1, 1970 to timeout
	unsigned period;			// period in uSec
	bool HasExpired(unsigned long long now);
	void UpdateTimeout(unsigned long long now);
	unsigned long long GetNextTimeout(unsigned long long curTimeout);

protected:
	virtual void Routine() = 0;

public:
	Callback(unsigned _period);
	virtual ~Callback()
	{}
};

// macros that help more clearly define the handler of a periodic routine
#define BEGIN_PERIODIC_ROUTINE(f) struct t_##f
#define END_PERIODIC_ROUTINE(f) ; PeriodicRoutine<t_##f> f

template <class T>
class PeriodicRoutine : public Callback
{
private:
	T* callbackCode;

protected:
	void Routine()
	{
		callbackCode->Routine();
	}

public:
	PeriodicRoutine(unsigned period) : Callback(period)
	{
		callbackCode = new T;
	}
	virtual ~PeriodicRoutine()
	{
		delete callbackCode;
	}
};

// base class for a sensor that must be polled
class GenericSensor : public Callback
{
protected:
	int fd;

public:
	GenericSensor(unsigned period) : Callback(period), fd(-1)
	{}
	virtual ~GenericSensor()
	{}
};

// base class for a sensor that sends an interrupt when data is available
class SelectableSensor
{
friend class Framework;

protected:
	int dataFd;
	virtual void Handler() = 0;

public:
	SelectableSensor() : dataFd(-1)
	{}
	virtual ~SelectableSensor()
	{}
};

// function dispatcher framework
class Framework
{
private:
	// list of sensors that are intrinsically selectable, i.e. those derived from DP_Sensor
	std::list<SelectableSensor*> selectableSensors;
	std::list<Callback*> periodicRoutines;
	int maxFd;
	timeval* ExecRoutines();

	int OpenConnection();

public:
	static int cmdfd;

	Framework();
	virtual ~Framework()
	{}

	static unsigned long long Tv2us(struct timeval *ptv);

	// register a sensor that can be selected
	void Register(SelectableSensor*);

	// register a sensor that must be polled
	void Register(Callback*);

	// the main function dispatcher
	void MainEventLoop();
};

void InitControlProgram(int argc, char* argv[], Framework& framework);
void Shutdown();
void Shutdown(const char* msg, int err);

#endif /* INCLUDE_FRAMEWORK_H_ */

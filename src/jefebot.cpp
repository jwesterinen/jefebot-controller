/*
 * jefebot.c
 * 
 * Description:  This is the control program for the jefebot.
 *   There are two modes that are selectable by the buttons:
 *     1. Roam: In this mode jefebot roams around a table without falling off.
 *        This is the first of the 3 HBRC Table Top challenges.
 *     2. GoToObject: In this mode jefebot finds an object on a table then pushes
 *        it off the table without itself falling.  This is the second of the 
 *        3 HBRC Table Top challenges.
 *   This module defines and registers all of the events and their handlers,
 *   including the two behavior controllers for the modes described above.  As
 *   described in the DP Framework project, everything including the specific
 *   control programs are events.
 * 
 * Synopsis:
 *     jefebot [-m<mode> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -d <distance> -a <angle> -v -h]
 *
 *     options:
 *         -m <mode>:     set the controller mode: 'r' = Roam, 'o' = GoToObject
 *         -e <value>:    set the range outside of which an edge is detected
 *         -o <value>:    set the range within which to find an object
 *         -i <value>:    set how close to stop at the object
 *         -s <value>:    set the motor speed (must be >=60)
 *         -p <value>:    print sensor values: 'v' = battery voltage, 's' = all distance sensors (range and edge)
 *         -d <value>:    move forward the specified number of centimeters
 *         -a <value>:    spin CW the specified number of radians
 *         -v:            set verbose mode
 *         -h:            display this help

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <unistd.h>
#include <getopt.h>
#include "roam_controller.h"
#include "goto_object_controller.h"

// control program errors
#define ERR_CONTROLLER_MODE		-2001
#define ERR_LOW_VOLTAGE			-2002

// readable timeout values
#define PERIOD_50_mSEC 50
#define PERIOD_100_mSEC 100
#define PERIOD_300_mSEC 300
#define PERIOD_500_mSEC 500
#define PERIOD_1_SEC 1000
#define PERIOD_10_SEC 10000

// battery constants
#define ADC_BATT_CHANNEL 7
#define BATTERY_CUTOFF_VOLTAGE 10.0
#define BatteryVoltage (4 * voltMeter->GetVoltage(ADC_BATT_CHANNEL))

// command line defaults
#define DEFAULT_SPEED 35.0
#ifdef USE_DISTANCE_NOT_VOLTAGE
#define DEFAULT_EDGE_LIMIT ???
#else
#define DEFAULT_EDGE_LIMIT 1000
#endif
#define DEFAULT_INNER_LIMIT 40
#define DEFAULT_OUTER_LIMIT 1000

// controller modes, i.e. behaviors
enum CONTROLLER_MODE {CM_ROAM, CM_GOTO_OBJECT, CM_GOTO_GOAL};

// command line options
struct Options
{
	bool isVerbose;
	bool isTestMode;
	bool doPrintBatteryVoltage;
	bool doPrintSensorValues;
	int distanceToMove;
	float angleToSpin;
	float defaultMotorSpeed;
	int nominalEdgeLimit;
	int objectInnerLimit;
	int objectOuterLimit;
	CONTROLLER_MODE controllerMode;

	Options() :
		isVerbose(false),
		isTestMode(false),
		doPrintBatteryVoltage(false),
		doPrintSensorValues(false),
		distanceToMove(0),
		angleToSpin(0.0),
		defaultMotorSpeed(DEFAULT_SPEED),
		nominalEdgeLimit(DEFAULT_EDGE_LIMIT),
		objectInnerLimit(DEFAULT_INNER_LIMIT),
		objectOuterLimit(DEFAULT_OUTER_LIMIT),
		controllerMode(CM_ROAM)
	{}
} options;

// jefebot elements
UserInterface* ui;
EdgeDetector* edgeDetector;
SinglePingRangeSensor* rangeSensor;
Locomotive* locomotive;
Controller* controller;
ADC* voltMeter;

// convert error code to error description string
const char* GetErrorMsg(int err)
{
	switch (err)
	{
    case ERR_NONE:                  return "";
    case ERR_INITIALIZATION:        return "initialization error";
    case ERR_READ:                  return "read error";
    case ERR_WRITE:                 return "write error";
    case ERR_SELECT:                return "select error";
    case ERR_PARAMS:                return "invalid parameters";
    case ERR_REGISTRATION:          return "callback registration error";
    case ERR_CONTROLLER_MODE:       return "invalid mode";
    case ERR_LOW_VOLTAGE:           return "!!LOW BATTERY VOLTAGE!!";
		default:
			break;
	}
	return "unknown error";
}

// ***** periodic event handler routines *****

// voltage watchdog routine to run every 10 Sec
BEGIN_PERIODIC_ROUTINE(VoltageWatchdog)

	// check the battery voltage and shutdown if less than the cutoff value
	if (BatteryVoltage < BATTERY_CUTOFF_VOLTAGE)
	{
		Shutdown("jefebot", ERR_LOW_VOLTAGE);
	}

END_PERIODIC_ROUTINE(VoltageWatchdog)(PERIOD_10_SEC);

// display the battery voltage
BEGIN_PERIODIC_ROUTINE(DisplayBatteryVoltage)

	if (BatteryVoltage == 0.0)
	{
		printf("jefebot: battery voltage reads 0 -- is it connected?\n");
		Shutdown("jefebot", ERR_LOW_VOLTAGE);
	}
	printf("battery voltage = %#.1f V\n", BatteryVoltage);
	Shutdown();

END_PERIODIC_ROUTINE(DisplayBatteryVoltage)(PERIOD_100_mSEC);

// periodic routine to show sensor values
BEGIN_PERIODIC_ROUTINE(DisplaySensorValues)

	try
	{
		printf("range value=%u  edge sensors: 1=%u 2=%u 3=%u\n",
			rangeSensor->GetDistance(),
			edgeDetector->GetEdgeSensorValue(EdgeDetector::LEFT),
			edgeDetector->GetEdgeSensorValue(EdgeDetector::FRONT),
			edgeDetector->GetEdgeSensorValue(EdgeDetector::RIGHT)
		);

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

END_PERIODIC_ROUTINE(DisplaySensorValues)(PERIOD_100_mSEC);

// periodic routine to test metered linear movement
BEGIN_PERIODIC_ROUTINE(MoveDistance)

	try
	{
		locomotive->MoveForward();
		if (locomotive->HasMovedDistance(options.distanceToMove))
		{
			Shutdown();
		}

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

END_PERIODIC_ROUTINE(MoveDistance)(PERIOD_100_mSEC);

// periodic routine to test angular movement
BEGIN_PERIODIC_ROUTINE(SpinAngle)

	try
	{
		locomotive->SpinCCW();
		if (locomotive->HasTurnedAngle(options.angleToSpin))
		{
			Shutdown();
		}

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

END_PERIODIC_ROUTINE(SpinAngle)(PERIOD_100_mSEC);

// periodic routine to test for the pressing of button S3 to shutdown
BEGIN_PERIODIC_ROUTINE(CheckInput)

	try
	{
		if (ui->IsButtonPressed(UserInterface::BUTTON3))
		{
			Shutdown();
		}

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

END_PERIODIC_ROUTINE(CheckInput)(PERIOD_100_mSEC);

// periodic routine to animate LEDs to indicate test mode
BEGIN_PERIODIC_ROUTINE(TestModeIndication)

	try
	{
		static unsigned char pattern = 0x55;
		ui->Display(pattern);
		pattern = ~pattern;

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

END_PERIODIC_ROUTINE(TestModeIndication)(PERIOD_300_mSEC);

// ***** initialization and termination *****

// parse the command line arguments
static void ParseOptions(int argc, char* argv[])
{
	const char* optStr = "m:e:o:i:s:p:d:a:vh";
	int opt;

	while ((opt = getopt(argc, argv, optStr)) != -1)
	{
		switch (opt)
		{
			case 'm':
				switch (optarg[0])
				{
					case 'o':
						options.controllerMode = CM_GOTO_OBJECT;
						break;
					case 'g':
						options.controllerMode = CM_GOTO_GOAL;
						break;
					case 'r':
						break;
					default:
						printf("usage: jefebot [-m<mode> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -d <distance> -a <angle> -v -h]\n");
						exit(ERR_CONTROLLER_MODE);
				}
				break;
			case 'e':
				options.nominalEdgeLimit = atoi(optarg);
				break;
			case 'o':
				options.objectInnerLimit = atoi(optarg);
				break;
			case 'i':
				options.objectOuterLimit = atoi(optarg);
				break;
			case 's':
				options.defaultMotorSpeed = atof(optarg);
				break;
			case 'p':
				options.isTestMode = true;
				switch (optarg[0])
				{
					case 'v':
						options.doPrintBatteryVoltage = true;
						break;
					case 's':
						options.doPrintSensorValues = true;
						break;
					default:
						printf("usage: jefebot [-m<mode> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -d <distance> -a <angle> -v -h]\n");
						exit(ERR_INITIALIZATION);
				}
				break;
			case 'd':
				options.isTestMode = true;
				options.distanceToMove = atoi(optarg);
				break;
			case 'a':
				options.isTestMode = true;
				options.angleToSpin = atof(optarg);
				break;
			case 'v':
				options.isVerbose = true;
				break;
			case 'h':
				printf("usage: jefebot [-m<mode> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -d <distance> -a <angle> -v -h]\n");
				printf("\n");
				printf("     options:\n");
				printf("         -m <mode>:     set the controller mode: 'r' = Roam, 'o' = GoToObject\n");
				printf("         -e <value>:    set the range outside of which an edge is detected\n");
				printf("         -o <value>:    set the range within which to find an object\n");
				printf("         -i <value>:    set how close to stop at the object\n");
				printf("         -s <value>:    set the motor speed (must be >=60)\n");
				printf("         -p <value>:    print sensor values: 'v' = battery voltage, 's' = all distance sensors (range and edge)\n");
				printf("         -d <value>:    move forward the specified number of centimeters\n");
				printf("         -a <value>:    spin CW the specified number of radians\n");
				printf("         -v:            set verbose mode\n");
				printf("         -h:            display this help\n");
				exit(ERR_NONE);
			default:
				printf("usage: jefebot [-m<mode> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -d <distance> -a <angle> -v -h]\n");
				exit(ERR_INITIALIZATION);
		}
	}
#if 0
	if (optind >= argc)
	{
		printf("jefebot: expected arg after opt\n");
		exit(ERR_INITIALIZATION);
	}
#endif
}

// init the behavior controll program defined in the command line
void InitControlProgram(int argc, char* argv[], DP::EventContext& evtCtx)
{
	try
	{
		// parse the command line options
		ParseOptions(argc, argv);

		// create the elements of jefebot that are required for all modes
		ui = new UserInterface(evtCtx);
		edgeDetector = new EdgeDetector(evtCtx, options.nominalEdgeLimit);
		rangeSensor = new SinglePingRangeSensor(evtCtx, options.objectInnerLimit, options.objectOuterLimit);
		voltMeter = new VoltMeter(evtCtx);
		locomotive = new Locomotive(evtCtx, options.defaultMotorSpeed);

		// register an input handler routine
		evtCtx.Register(&CheckInput);

		// register a battery voltage monitoring routine
		evtCtx.Register(&VoltageWatchdog);

		// perform test mode activities
		if (options.isTestMode)
		{
			// register a periodic routine to indicate test mode via LED animation
			evtCtx.Register(&TestModeIndication);

			// register a periodic routine to display the battery voltage
			if (options.doPrintBatteryVoltage)
			{
				evtCtx.Register(&DisplayBatteryVoltage);
			}

			// register a periodic routine to display sensor values
			else if (options.doPrintSensorValues)
			{
				evtCtx.Register(&DisplaySensorValues);
			}

			// register a periodic routine to display sensor values
			else if (options.distanceToMove != 0)
			{
				evtCtx.Register(&MoveDistance);
				printf("moving %d cm...\n", options.distanceToMove);
			}

			// register a periodic routine to display sensor values
			else if (options.angleToSpin != 0.0)
			{
				evtCtx.Register(&SpinAngle);
				printf("spinning %f radians...\n", options.angleToSpin);
			}
		}

		// create the rest of the elements needed for normal operation
		else
		{
			// init State machine
			Controller::Context ctx(*ui, *locomotive, *edgeDetector, *rangeSensor);
			switch(options.controllerMode)
			{
				case CM_ROAM:
					controller = new RoamController(ctx, options.isVerbose);
					evtCtx.Register(controller);
					break;
				case CM_GOTO_OBJECT:
					controller = new GotoObjectController(ctx, options.isVerbose);
					evtCtx.Register(controller);
					break;
				default:
					assert(false);
			}
		}

	} catch (DP::FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

}

// default shutdown routine
void Shutdown()
{
	Shutdown("", ERR_NONE);
}

// routine to properly shut down jefebot
void Shutdown(const char* msg, int error)
{
	// stop moving if there is a locomotive
	if (locomotive)
		locomotive->Stop();

	// clear LEDs
	ui->Display(0);

	//shutdown any SPI or I2C devices

	// display shutdown status message
	if (error != ERR_NONE)
		printf("%s: %s\nexiting...\n", msg, GetErrorMsg(error));
	else
		printf("%s\n", msg);

	// allow dpserver to catch up
    sleep(1);

	// release all objects
    delete ui;
    delete locomotive;
    delete edgeDetector;
    delete rangeSensor;
    delete voltMeter;

    exit(error);
}


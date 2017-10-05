/*
 * jefebot.c
 * 
 * Description:  This is the control program for the jefebot.
 *   There are two modes that are selectable by the buttons:
 *     1. Roam: In this mode the jefebot moves forward until it detects an edge
 *        then avoids it and continues to move forward.
 *     2. GoToObject: In this mode the jefebot spins until it detects an object
 *        then it moves towards the object and stops when it arrives.
 * 
 * Synopsis:
 *     jefebot [-m<r|o|g> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -v -h]
 *
 *     options:
 *         -m <mode>: 	set the controller mode:
 *         				 'r' = roam (default)
 *         				 'o' = go to object
 *         				 'g' = go to goal (unimplemented)
 *         -e <value>:	set the range outside of which an edge is detected
 *         -o <value>: 	set the range within which to find an object
 *         -i <value>:	set how close to stop at the object
 *         -s <value>:	set the motor speed (must be >=60)
 *         -p <value>:	print sensor values:
 *                       'v' = battery voltage
 *                       's' = all distance sensors (range and edge)
 *         -v:          set verbose mode
 *         -h:			display help
 */

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <unistd.h>
#include <getopt.h>
#include "controller.h"

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
#define DEFAULT_SPEED 40.0
#ifdef USE_DISTANCE_NOT_VOLTAGE
#define DEFAULT_EDGE_LIMIT ???
#else
#define DEFAULT_EDGE_LIMIT 1000
#endif
//#define DEFAULT_INNER_LIMIT 40
#define DEFAULT_INNER_LIMIT 35
//#define DEFAULT_OUTER_LIMIT 260
#define DEFAULT_OUTER_LIMIT 300

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

// ***** periodic routines *****

// voltage watchdog routine to run every 10 Sec
BEGIN_PERIODIC_ROUTINE(VoltageWatchdog)
{
	void Routine()
	{
		// check the battery voltage and shutdown if less than the cutoff value
		if (BatteryVoltage < BATTERY_CUTOFF_VOLTAGE)
		{
			Shutdown("jefebot", ERR_LOW_VOLTAGE);
		}
	}
}
END_PERIODIC_ROUTINE(VoltageWatchdog)(PERIOD_10_SEC);

BEGIN_PERIODIC_ROUTINE(DisplayBatteryVoltage)
{
	void Routine()
	{
        if (BatteryVoltage == 0.0)
		{
			printf("jefebot: battery voltage reads 0 -- is it connected?\n");
			Shutdown("jefebot", ERR_LOW_VOLTAGE);
		}
		printf("battery voltage = %#.1f V\n", BatteryVoltage);
		Shutdown();
	}
}
END_PERIODIC_ROUTINE(DisplayBatteryVoltage)(PERIOD_100_mSEC);

// periodic routine to show sensor values
BEGIN_PERIODIC_ROUTINE(DisplaySensorValues)
{
	void Routine()
	{
		try
		{
			printf("range value=%u  edge sensors: 1=%u 2=%u 3=%u\n",
				rangeSensor->GetDistance(),
				edgeDetector->GetEdgeSensorValue(EdgeDetector::LEFT),
				edgeDetector->GetEdgeSensorValue(EdgeDetector::FRONT),
				edgeDetector->GetEdgeSensorValue(EdgeDetector::RIGHT)
			);

		} catch (FrameworkException& e) {
			Shutdown(e.what(), e.Error());
		}
	}
}
END_PERIODIC_ROUTINE(DisplaySensorValues)(PERIOD_100_mSEC);

// periodic routine to show sensor values
BEGIN_PERIODIC_ROUTINE(MoveDistance)
{
	void Routine()
	{
		try
		{
			locomotive->MoveForward(0);
			if (locomotive->HasMovedDistance(options.distanceToMove))
			{
				Shutdown();
			}

		} catch (FrameworkException& e) {
			Shutdown(e.what(), e.Error());
		}
	}
}
END_PERIODIC_ROUTINE(MoveDistance)(PERIOD_100_mSEC);

// periodic routine to show sensor values
BEGIN_PERIODIC_ROUTINE(SpinAngle)
{
	void Routine()
	{
		try
		{
			locomotive->SpinCCW(0);
			if (locomotive->HasTurnedAngle(options.angleToSpin))
			{
				Shutdown();
			}

		} catch (FrameworkException& e) {
			Shutdown(e.what(), e.Error());
		}
	}
}
END_PERIODIC_ROUTINE(SpinAngle)(PERIOD_100_mSEC);

// periodic routine to test for the pressing of button S3 to shutdown
BEGIN_PERIODIC_ROUTINE(CheckInput)
{
	void Routine()
	{
		try
		{
			if (ui->IsButtonPressed(UserInterface::BUTTON3))
			{
				Shutdown();
			}

		} catch (FrameworkException& e) {
			Shutdown(e.what(), e.Error());
		}
	}
}
END_PERIODIC_ROUTINE(CheckInput)(PERIOD_100_mSEC);

// periodic routine to animate LEDs to indicate test mode
BEGIN_PERIODIC_ROUTINE(TestModeIndication)
{
	void Routine()
	{
		try
		{
			static unsigned char pattern = 0x55;
			ui->Display(pattern);
			pattern = ~pattern;

		} catch (FrameworkException& e) {
			Shutdown(e.what(), e.Error());
		}
	}
}
END_PERIODIC_ROUTINE(TestModeIndication)(PERIOD_300_mSEC);

// ***** initialization and termination *****

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
				printf("         -m <mode>:     set the controller mode: 'p' = Roam, 'o' = GoToObject\n");
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
				printf("usage: jefebot [-m<r|o|g> -e <edge thresh> -o <obj outer> -i <obj inner> -s <speed> -p<v|s> -v -h]\n");
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

void InitControlProgram(int argc, char* argv[], Framework& framework)
{
	try
	{
		// parse the command line options
		ParseOptions(argc, argv);

		// create the elements of jefebot that are required for all modes
		ui = new UserInterface(framework);
		edgeDetector = new EdgeDetector(framework, options.nominalEdgeLimit);
		rangeSensor = new SinglePingRangeSensor(framework, options.objectInnerLimit, options.objectOuterLimit);
		voltMeter = new VoltMeter(framework);
		locomotive = new Locomotive(framework, options.defaultMotorSpeed);

		// register an input handler routine
		framework.Register(&CheckInput);

		// register a battery voltage monitoring routine
		framework.Register(&VoltageWatchdog);

		// perform test mode activities
		if (options.isTestMode)
		{
			// register a periodic routine to indicate test mode via LED animation
			framework.Register(&TestModeIndication);

			// register a periodic routine to display the battery voltage
			if (options.doPrintBatteryVoltage)
			{
				framework.Register(&DisplayBatteryVoltage);
			}

			// register a periodic routine to display sensor values
			else if (options.doPrintSensorValues)
			{
				framework.Register(&DisplaySensorValues);
			}

			// register a periodic routine to display sensor values
			else if (options.distanceToMove != 0)
			{
				framework.Register(&MoveDistance);
				printf("moving %d cm...\n", options.distanceToMove);
			}

			// register a periodic routine to display sensor values
			else if (options.angleToSpin != 0.0)
			{
				framework.Register(&SpinAngle);
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
					framework.Register(controller);
					break;
				case CM_GOTO_OBJECT:
					controller = new GotoObjectController(ctx, options.isVerbose);
					framework.Register(controller);
					break;
				default:
					assert(false);
			}
		}

	} catch (FrameworkException& e) {
		Shutdown(e.what(), e.Error());
	}

}

void Shutdown()
{
	Shutdown("", ERR_NONE);
}

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


/*
 * dp_peripherals.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: jeff
 */

#include <cstdio>
#include <unistd.h>
#include <cerrno>
#include <cassert>
#include "framework.h"
#include "dp_peripherals.h"

void DP_Peripheral::WriteConfig(std::string resource, const std::string& arg)
{
	std::string commandStr = "dpset " + id + " " + resource;
    WriteCommand(commandStr, arg);
}

void DP_Peripheral::WriteCommand(std::string command, std::string arg)
{
	std::string commandLine = command + " " + arg + "\n";
    unsigned wrsize = write(cmdfd, commandLine.c_str(), commandLine.length());
    if (wrsize != commandLine.length())
    {
    	throw FrameworkException("WriteCommand", ERR_WRITE);
    }
}

void DP_Sensor::StartDataStream()
{
	std::string commandLine = "dpcat " + id + " " + dataRsc + "\n";
    unsigned wrsize = write(dataFd, commandLine.c_str(), commandLine.length());
    if (wrsize != commandLine.length())
    {
    	throw FrameworkException("StartDataStream", ERR_WRITE);
    }
}

// FIXME: need to resolve how to use this one fct for both command replies and data streams
void DP_Sensor::ReadResponse(char* buffer, int bufsize)
{
    int bufptr;
    int retval;

    for (bufptr = 0; bufptr < bufsize; bufptr++)
    {
        retval = read(dataFd, &buffer[bufptr], 1);
        if (retval < 0)
        {
            if (retval != EAGAIN && retval != EINTR)
            {
            	throw FrameworkException("ReadResponse", ERR_READ);
            }
            else
                continue;
        }
        else if (retval == 0)
        {
        	throw FrameworkException("ReadResponse", ERR_READ);
        }
        if (buffer[bufptr] == '\n')
        {
        	// check if this newline is NOT part of a command response, <nl><\>
        	if ((bufptr < bufsize-1) && (buffer[bufptr+1] != '\\'))
			{
        		// terminate the data string
				buffer[bufptr] = 0;
				break;
			}
        }
        else if (buffer[bufptr] == '\\')
        {
            // terminate the command response string, <nl><\>
        	buffer[bufptr] = 0;
        	break;
        }
    }
    if (bufptr == bufsize)
    {
    	throw FrameworkException("ReadResponse", ERR_READ);
    }
}

void DP_Bb4io::SetLeds(unsigned char pattern)
{
	char buffer[80] = {0};
    sprintf(buffer, "%x", pattern);
	WriteConfig("leds", buffer);
}

void DP_Bb4io::Handler()
{
	char buffer[80] = {0};
    int paramQty = 0;

    ReadResponse(buffer, sizeof(buffer));
    paramQty = sscanf(buffer, "%u", &buttons);
    if (paramQty != 0 && paramQty != 1)
    {
    	throw FrameworkException("DP_BB4IO", ERR_RESPONSE);
    }
}

unsigned DP_Ping4::GetDistance(unsigned channelId)
{
	switch (channelId)
	{
		case SENSOR_0:
			return distances[0];
			break;
		case SENSOR_1:
			return distances[1];
			break;
		case SENSOR_2:
			return distances[2];
			break;
		case SENSOR_3:
			return distances[3];
			break;

		// should never happen
		default:
			assert(false);
	}
	return 0;
}

void DP_Ping4::Handler()
{
    char buffer[80] = {0};
    unsigned distance;
    int id;

    ReadResponse(buffer, sizeof(buffer));
    if (sscanf(buffer, "%d %d", &id, &distance) != 2)
    {
    	throw FrameworkException("DP_Ping4", ERR_RESPONSE);
    }
    distances[id] = distance;
}

void DP_Count4::SetUpdateRate(unsigned rate)
{
	// the update period must be between 10 and 60 milliseconds in steps of 10 milliseconds
	if (rate != 10 && rate != 20 && rate != 30 && rate != 40 && rate != 50 && rate != 60)
	{
		throw FrameworkException("DP_Count4", ERR_PARAMS);
	}
	WriteConfig("update_rate", ToString(rate));
}
void DP_Count4::SetEdges(unsigned setting1, unsigned setting2, unsigned setting3, unsigned setting4)
{
	std::string argStr;
	argStr = ToString(setting1) + " " + ToString(setting2) + " " + ToString(setting3) + " " + ToString(setting4);
    WriteConfig("edges", argStr);
}
void DP_Count4::Handler()
{
    char buffer[80] = {0};

    // read the tick counts
    ReadResponse(buffer, sizeof(buffer));
    if (sscanf(buffer, "%d %f %d %f %d %f %d %f",
        &counts[0], &intervals[0],
        &counts[1], &intervals[1],
        &counts[2], &intervals[2],
        &counts[3], &intervals[3]
    ) != 8)
    {
    	throw FrameworkException("DP_Count4", ERR_RESPONSE);
    }
}

void DP_Adc812::Config(unsigned period, DIFF_PAIRS differentialPairs)
{
	char buffer[80] = {0};
	sprintf(buffer, "%u, 0x%02x", period, differentialPairs);
    WriteConfig("enable", buffer);
}

void DP_Adc812::Handler()
{
    char buffer[80] = {0};

    // read the tick counts
    ReadResponse(buffer, sizeof(buffer));
    if (sscanf(buffer, "%x %x %x %x %x %x %x %x",
        &samples[0], &samples[1],&samples[2], &samples[3],
		&samples[4], &samples[5], &samples[6], &samples[7]
    ) != 8)
    {
    	throw FrameworkException("DP_Adc812", ERR_RESPONSE);
    }
}

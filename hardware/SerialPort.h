//######################################################################
//
// This file is part of GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: 
//
//######################################################################

#ifndef _serialport_h_
#define _serialport_h_

#include "windows.h"
#include <time.h>

void sleep(clock_t wait);

class SerialPort{
public:
	enum PortNumber{COM1, COM2, COM3, COM4, COM5, COM6};
private:
	DCB dcb;
	HANDLE hCom;
	int lastError;

public:
	SerialPort(PortNumber port, int baudRate);
	~SerialPort();
	int getError(){return lastError;}

	int writeString(char *string);
	int writeNBytes(int nBytes, unsigned char *record);
	int readString(int nBytes, char *string);
	int readTerminatedString(int nBytes, char *string);
	int readStringWithTerminator(int nBytes, char *string, char terminator);
	int readNBytes(int nBytes, unsigned char *string);
	void useTimeouts();
};

#endif
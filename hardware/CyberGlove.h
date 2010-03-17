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

#ifndef _cyberglove_h_
#define _cyberglove_h_

#include "SerialPort.h"

#define N_SENSOR_VALUES 24

class CyberGlove {
private:
	SerialPort *port;
	unsigned char *sensorValues;
public:
	CyberGlove();
	~CyberGlove();

	int testGlove();

	int instantRead();
	unsigned char getSensorValue(int i);
	const unsigned char *getAllValues(){return sensorValues;}
};

#endif
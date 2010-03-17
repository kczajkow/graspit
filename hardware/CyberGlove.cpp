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

#include "CyberGlove.h"
#include <stdio.h>

CyberGlove::CyberGlove()
{
	port = new SerialPort(SerialPort::COM6, 38400);
	if ( port->getError() ) {
		fprintf(stderr,"Raw Cyber Glove failed to initialize serial port\n");
	}

	sensorValues = new unsigned char[N_SENSOR_VALUES];
}

CyberGlove::~CyberGlove()
{
	delete port;
	delete [] sensorValues;
}

int CyberGlove::testGlove()
{
	if (!port->writeString("?G") ) {
		fprintf(stderr,"Write failed\n");
		return 0;
	}

	char* result = new char[20];
	int read = port->readTerminatedString(20, result);
	if (read <= 0) {
		fprintf(stderr,"No answer read!\n");
		return 0;
	}
	if (result[read-1] != 0) {
		fprintf(stderr,"last bit is not 0:\n");
		return 0;
	}

	int r;
	switch(result[2]) {
		case 0:
			fprintf(stderr,"No glove and not initialized\n");
			r = 0;
			break;
		case 1:
			fprintf(stderr,"Glove present but not initialized\n");
			r = 0;
			break;
		case 2:
			fprintf(stderr,"Glove not present but initialized (strange...)\n");
			r = 0;
			break;
		case 3:
			r = 1;
			break;
		default:
			fprintf(stderr,"Unknown value returned: %d %c\n",result[3], result[3]);
			r = 0;
			break;
	}
	delete [] result;
	return r;
}

int CyberGlove::instantRead()
{
	port->writeString("G");

	char result[N_SENSOR_VALUES + 5];
	int nRead = port->readTerminatedString(N_SENSOR_VALUES + 5, result);
	if (nRead != N_SENSOR_VALUES) {
		fprintf(stderr,"Wrong number of sensor values returned!\n");
		return 0;
	}
	memcpy(sensorValues, result+1, N_SENSOR_VALUES-1);
	return 1;
}

unsigned char CyberGlove::getSensorValue(int i)
{
	if (i<0 || i>=N_SENSOR_VALUES) {
		fprintf(stderr,"Sensor id out of range!\n");
		return 0;
	}
	return sensorValues[i];	
}
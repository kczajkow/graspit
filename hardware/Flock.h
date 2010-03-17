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

#ifndef _flock_h_
#define _flock_h_

#include "SerialPort.h"

class Flock {
public:
	enum opMode{POS_ANG, POS_MAT};
private:
	int nBirds;
	SerialPort *port;

	opMode currentOpMode;
	double angles[3];
	double position[3];
	double rotationMatrix[9];

	bool relativeOrientation;
	double basePosition[3];
	double baseAngles[3];

	void angleAlign(double *a);

	int translateRecord(int nBytes, unsigned char *record, unsigned short *words);
	int encodeRecord(int nWords, unsigned char *record, unsigned short *words);
	int readAndTranslate(int nBytes, unsigned char *record, unsigned short *words);

	void talkToBird(int b);
	void initFlock();
public:
	Flock(int n=1);
	~Flock();

	virtual int instantRead(int b=1);

	virtual void getPosition(double *p){memcpy(p, position, 3*sizeof(double));}
	virtual void getAngles(double *p){memcpy(p, angles, 3*sizeof(double));}
	virtual void getRotationMatrix(double *p){memcpy(p, rotationMatrix, 9*sizeof(double));}

	int setOpMode(opMode m);
	int setRelativeFrame(int b=1);
	int setAbsoluteFrame();
};

#endif
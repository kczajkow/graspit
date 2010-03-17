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

#ifndef _glovetrans_h_
#define _glovetrans_h_

#include "CyberGlove.h"

class GloveTrans {
private:
	CyberGlove *rawGlove;
	float *slopes, *intercepts;
	float *jointValues;
public:
	GloveTrans();
	~GloveTrans();
	void setGlove(CyberGlove *glove){rawGlove = glove;}

	int instantRead();
	float getJointValue(int i);
	int testGlove(){return rawGlove->testGlove();}
	int getNumSensors(){return N_SENSOR_VALUES;}
	void setParameters(int s, float sMin, float sMax, float dMin, float dMax);
};

#endif
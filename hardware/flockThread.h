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

#include "flock.h"

class FlockThread : public Flock
{
protected:
	HANDLE mMutex;
	HANDLE mThreadHandle;
	DWORD mThreadId;
	bool mDone;
	bool mNewData;

	virtual int instantRead(int b=1);
	friend DWORD WINAPI FlockRunThread(void*);
public:
	FlockThread(int n=1) : Flock(n){mMutex = CreateMutex(NULL, FALSE, NULL);mThreadHandle = NULL;}

	virtual void getPosition(double *p);
	virtual void getAngles(double *p);
	virtual void getRotationMatrix(double *p);

	bool isDone();
	bool newData();
	void startThread();
	void stopThread();
	bool isFinished();
};

DWORD WINAPI FlockRunThread(void *);

class Foo
{
public:
	static void run(void *){}
};
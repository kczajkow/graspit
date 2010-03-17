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

#include "windows.h"
#include "BarrettHand.h"

/*
	This class is intended to do blocking Barrett communication, so we know when the hand has finished whatever
	it was doing. However, for now it does not have everything implemented like it should
*/
class BarrettHandThread : public BarrettHand{
protected:
	enum Status{IDLE, READY, BUSY, DEAD};
	enum ThreadCommands{MOVE_TOGETHER, CLOSE, OPEN};

	Status mStatus;
	BarrettHand *mRealHand;
	HANDLE mMutex, mSemaphore, mThreadHandle;
	DWORD mThreadId;

	ThreadCommands mCurrentCommand;
	double mDof[4];
	int mCommandFingers;
	bool mNewCommand;
	bool mHandReady;
	bool mDone;

public:	
	BarrettHandThread();
	~BarrettHandThread();

	void MoveTogether(double *destinations);
	void Open(int fingers);
	void Close(int fingers);
	bool isBusy();

	void threadLoop();
	void startThread();
	void stopThread();
	bool isFinished();
};

DWORD WINAPI BarrettRunThread(void *);

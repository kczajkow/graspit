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
// $Id: graspCaptureDlg.cpp,v 1.4.2.1 2009/04/24 22:19:24 cmatei Exp $
//
//######################################################################

#include "graspCaptureDlg.h"

#include <QFileDialog>

#include "world.h"
#include "body.h"
#include "robot.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "graspRecord.h"
#include "gloveInterface.h"
#include "quality.h"
#include "grasp.h"

void GraspCaptureDlg::init()
{
	mSelectedObject = -1;
	graspQualityBar->setPercentageVisible(FALSE);
	graspQualityBar->setTotalSteps(1000);
}

void GraspCaptureDlg::exitButton_clicked()
{
	clearObjectListButton_clicked();
	clearPosesButton_clicked();
	delete mQual;
	QDialog::accept();
}


void GraspCaptureDlg::setWorld( World *w )
{
	mWorld = w;
	mHand = w->getCurrentHand();
	if (mHand == NULL) {
		QTWARNING("No hand selected; will crash");
	}

	mQual = new QualEpsilon( mHand->getGrasp(), QString("Capture_dlg_qm"),"L1 Norm");
	fprintf(stderr,"Grasp quality measure created\n");

}


void GraspCaptureDlg::addWorldObject( int objIndex )
{
	mBodies[objIndex]->addToIvc();
	mWorld->addBody(mBodies[objIndex]);
}


void GraspCaptureDlg::removeWorldObject( int objIndex )
{
//	fprintf(stderr,"Removing object %d\n",objIndex);
	graspItGUI->getIVmgr()->deselectBody( mBodies[objIndex] );
	mWorld->destroyElement( mBodies[objIndex], false );
}

void GraspCaptureDlg::objectSelectionBox_activated( const QString & )
{
	int newSelection = objectSelectionBox->currentItem();
	if (newSelection == mSelectedObject) return;
	if (mSelectedObject >= 0)
		removeWorldObject(mSelectedObject);
	mSelectedObject = newSelection;
	addWorldObject(mSelectedObject);
}

void GraspCaptureDlg::prevObjectButton_clicked()
{
	if ( (int)mBodies.size() <= 1) return;
	if (mSelectedObject >= 0) {
		removeWorldObject(mSelectedObject);
	}
	mSelectedObject--;
	if (mSelectedObject< 0)
		mSelectedObject = (int)mBodies.size() - 1;
	addWorldObject(mSelectedObject);
	objectSelectionBox->setCurrentItem(mSelectedObject);
}


void GraspCaptureDlg::nextObjectButton_clicked()
{
	if ( (int)mBodies.size() <= 1) return;
	if (mSelectedObject >= 0) {
		removeWorldObject(mSelectedObject);
	}
	mSelectedObject++;
	if (mSelectedObject >= (int)mBodies.size())
		mSelectedObject = 0;
	addWorldObject(mSelectedObject);
	objectSelectionBox->setCurrentItem(mSelectedObject);
}


void GraspCaptureDlg::openObjectFolderButton_clicked()
{
	QStringList files = QFileDialog::getOpenFileNames(this,"Select object files to load", 
		QString(getenv("GRASPIT"))+QString("/models/objects"),"Objects (*.iv *.wrl)");

	GraspableBody *newBody;
	QStringList::iterator it;
	for (it = files.begin(); it!=files.end(); it++) {
		QString name(*it);
		name = name.section('/',-1,-1);
		name = name.section('.',0,0);
		newBody = new GraspableBody(mWorld, name);

		if ( newBody->load(*it) == FAILURE ) {
			fprintf(stderr,"Failed to load body: %s\n",name.latin1());
			delete newBody;
			continue;
		}

		mBodies.push_back(newBody);
		mFilenames.push_back(*it);
		objectSelectionBox->insertItem(name);
	}

	if ( mSelectedObject<0 && !mBodies.empty() ) {
		mSelectedObject = 0;
		objectSelectionBox->setCurrentItem(0);
		addWorldObject(mSelectedObject);
	}
}


void GraspCaptureDlg::clearObjectListButton_clicked()
{
	if (mSelectedObject >= 0)
		removeWorldObject(mSelectedObject);
	std::vector<GraspableBody*>::iterator it;
	for (it = mBodies.begin(); it!=mBodies.end(); it++) {
		(*it)->getIVGeomRoot()->unref();
		delete (*it);
	}
	mBodies.clear();
	mFilenames.clear();
	objectSelectionBox->clear();
	mSelectedObject = -1;
}

void GraspCaptureDlg::instantCaptureButton_clicked()
{
	GraspRecord *newGrasp = new GraspRecord( mHand->getNumDOF() );

	double *dofVals = new double[mHand->getNumDOF()];
	mHand->getDOFVals(dofVals);
	newGrasp->mPose->setAllJointValues(dofVals);
	newGrasp->mRobotName = mHand->getName();
	if (mSelectedObject < 0) {
		newGrasp->mObjectName = QString("no_object_used");
		newGrasp->mTran = transf::IDENTITY;
	} else {
		newGrasp->mObjectName = mFilenames[mSelectedObject];
//		newGrasp->mTran = mHand->getTran().inverse() * mBodies[mSelectedObject]->getTran();
		newGrasp->mTran = mBodies[mSelectedObject]->getTran() * mHand->getTran().inverse();
	}


	mGrasps.push_back(newGrasp);
	numPosesLabel->setNum((int)mGrasps.size());

	delete [] dofVals;
}

void GraspCaptureDlg::streamingButton_clicked()
{

}

void GraspCaptureDlg::savePosesButton_clicked()
{
 if (mGrasps.empty()) return;

	QString fn = QFileDialog::getSaveFileName(this, QString(), QString(getenv("GRASPIT"))+QString("/models/grasps"),
				"Grasp Files (*.grp)");
    if ( !fn.isEmpty() ) {
	    if (fn.section('.',1).isEmpty())
			fn.append(".grp");
	}
	writeGraspListToFile(&mGrasps, fn.latin1());
}

void GraspCaptureDlg::clearPosesButton_clicked()
{
	std::vector<GraspRecord*>::iterator it;
	for (it = mGrasps.begin(); it!= mGrasps.end(); it++) {
		delete (*it);
	}
	mGrasps.clear();
	numPosesLabel->setNum(0);
}

void GraspCaptureDlg::updateGraspQuality()
{
	double q = mQual->evaluate();
	if (q < 0) q = 0;
	QString qs;
	qs.setNum(q);
	qs.truncate(5);
	graspQualityLabel->setText(qs);
	graspQualityBar->setProgress((int)(q*1000));
	int c = mHand->getGrasp()->getNumContacts();
	numContactsLabel->setNum(c);
}

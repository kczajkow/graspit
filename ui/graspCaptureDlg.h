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
// $Id: graspCaptureDlg.h,v 1.3.2.2 2009/04/27 14:33:16 cmatei Exp $
//
//######################################################################

#ifndef _graspcapturedlg_h_
#define _graspcapturedlg_h_

#include "ui_graspCaptureDlg.h"

#include <QDialog>
#include <QString>

#include <vector>

class QualityMeasure;
class Hand;
class World;
class GraspRecord;
class GraspableBody;

//! Allows the user to record and save grasps of many objects, usually with the CyberGlove
/*! This dialog is intended to allow the user to cycle through a list
	of objects and record grasps for each of them. It is expected to
	be used to gather data with the CyberGlov: the user can cycle 
	through a list of objects, create grasps using the glove, and 
	save the grasps at the end. However, it is not dependent of the 
	glove and grasps set in any way can be recorded and saved.

	The main functionality is to specify a set of object files, then
	let GraspIt load them and cycle through them while recording grasps
	that can be later saved. It also allows the quality measures to be
	computed as grasps are created.

	The format in which grasps are saved is obsolete. In general, this
	dialog has not been used much, and its interface is obsolete and in
	need of a re-write.
*/
class GraspCaptureDlg : public QDialog, public Ui::GraspCaptureDlgUI
{
	Q_OBJECT
private:
    QualityMeasure *mQual;
    Hand* mHand;
    World* mWorld;
    int mSelectedObject;
    std::vector<GraspRecord*> mGrasps;
    std::vector<QString> mFilenames;
    std::vector<GraspableBody*> mBodies;

	void init();
	void addWorldObject( int objIndex );
	void removeWorldObject( int objIndex );

public:
	GraspCaptureDlg(QWidget *parent = 0) : QDialog(parent) {
		setupUi(this);
		init();
	}
	void setWorld( World *w );

public slots:
	void objectSelectionBox_activated( const QString & );
	void prevObjectButton_clicked();
	void nextObjectButton_clicked();
	void openObjectFolderButton_clicked();
	void clearObjectListButton_clicked();
	void instantCaptureButton_clicked();
	void streamingButton_clicked();
	void savePosesButton_clicked();
	void clearPosesButton_clicked();
	void exitButton_clicked();
	void updateGraspQuality();
};

#endif

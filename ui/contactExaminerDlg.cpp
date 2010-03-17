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
// $Id: contactExaminerDlg.cpp,v 1.6.2.1 2009/04/24 22:19:24 cmatei Exp $
//
//######################################################################

#include "contactExaminerDlg.h"

#include <QFileDialog>

#include "world.h"
#include "robot.h"
#include "contact.h"
#include "grasp.h"
#include "quality.h"
#include "eigenGrasp.h"
#include "graspitGUI.h"
#include "ivmgr.h"

void ContactExaminerDlg::markButton_clicked()
{
 int f,l;
 std::list<Contact *>::iterator cp;
 std::list<Contact *> contactList;
 Contact *newContact;

 //the distinctive sign of a virtual contact is that it is its own mate...
 mHand->showVirtualContacts(true);
 contactList = mHand->getPalm()->getContacts();
 for (cp=contactList.begin();cp!=contactList.end();cp++) {
  if ( (*cp)->getMate() != (*cp) ) {
   newContact = new VirtualContact(-1, 0, *cp);
   ((VirtualContact*)newContact)->setBody( mHand->getPalm() );
			mContacts.push_back(newContact);
			mHand->getPalm()->addVirtualContact(newContact);
		}
	}

	for(f=0;f<mHand->getNumFingers();f++) {
	    for (l=0;l<mHand->getFinger(f)->getNumLinks();l++) {
			contactList = mHand->getFinger(f)->getLink(l)->getContacts();
			for (cp=contactList.begin();cp!=contactList.end();cp++){
				if ( (*cp)->getMate() != (*cp) ) {
					newContact = new VirtualContact(f, l, *cp);
					((VirtualContact*)newContact)->setBody( mHand->getFinger(f)->getLink(l) );
					mContacts.push_back(newContact);
					mHand->getFinger(f)->getLink(l)->addVirtualContact(newContact);
				}
			}
		}
	}
	markedLabel->setNum( (int)mContacts.size() );
}


void ContactExaminerDlg::loadButton_clicked()
{
	QStringList fn = QFileDialog::getOpenFileNames( this, "Select virtual contact files to load",
		QString(getenv("GRASPIT"))+QString("/models/virtual"),"Virtual Contacts (*.vgr)" );
			
	QStringList::iterator it;
	for (it=fn.begin(); it!=fn.end(); it++) {
		loadContactFile(*it);
	}

	mContacts.clear(); //this leaks memory; should really fix this entire dialog
	int f,l;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;

	contactList = mHand->getPalm()->getVirtualContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		mContacts.push_back(*cp);
	}

	for(f=0;f<mHand->getNumFingers();f++) {
	    for (l=0;l<mHand->getFinger(f)->getNumLinks();l++) {
			contactList = mHand->getFinger(f)->getLink(l)->getVirtualContacts();
			for (cp=contactList.begin();cp!=contactList.end();cp++){
				mContacts.push_back(*cp);
			}
		}
	}

	markedLabel->setNum( (int)mContacts.size() );
	if (!mContacts.empty()) mHand->showVirtualContacts(true);
}

void ContactExaminerDlg::loadContactFile( QString file )
{
	if (mHand->loadContactData(file)) {
		//fprintf(stderr,"Contact file %s loaded\n",file.latin1());
	} else {
		//fprintf(stderr,"Contact file %s failed to load\n",file.latin1());
	}
}


void ContactExaminerDlg::saveButton_clicked()
{
	QString fn = QFileDialog::getSaveFileName( this, "Select filename", 
				 QString(getenv("GRASPIT"))+QString("/models/virtual"),"Virtual Grasp Files (*.vgr)" );
    if ( !fn.isEmpty() ) {
     if (fn.section('.',1).isEmpty())
   fn.append(".vgr");
 }

 //save...
 FILE *fp = fopen(fn.latin1(), "w");
 if (!fp) {
  fprintf(stderr,"Failed to open file for writing\n");
 }

 fprintf(fp,"%s\n",mHand->getName().latin1());
 fprintf(fp,"%d\n",(int)mContacts.size());
 for (int i=0; i<(int)mContacts.size(); i++) {
  ((VirtualContact*)mContacts[i])->writeToFile(fp);
 }
 fclose(fp);
}


void ContactExaminerDlg::exitButton_clicked()
{
 QDialog::accept();
}


void ContactExaminerDlg::init()
{
	mQual = NULL;
}

bool ContactExaminerDlg::setWorld( World *w )
{
	mWorld = w;
	mHand = mWorld->getCurrentHand();
	if (!mHand) return false;
	//this is now done strictly when it's needed
	//	mHand->getGrasp()->setObject(NULL);
	mQual = new QualEpsilon( mHand->getGrasp(), QString("Virtual_grasp_qm"),"L1 Norm");
	fprintf(stderr,"Grasp quality measure created\n");
	return true;
}


void ContactExaminerDlg::setHand( Hand *h )
{
	//if just a hand is set, then this dialog can olny load / mark virtual contacts
	//can not be used to sample quality

	mHand = h;
	mWorld = mHand->getWorld();
	sampleButton->setEnabled(false);
	updateQualityButton->setEnabled(false);
}

void ContactExaminerDlg::setVirtualCentroid()
{
	vec3 cog(0,0,0);
	position pos;
	for (int i=0; i<(int)mContacts.size(); i++) {
		pos = ((VirtualContact*)mContacts[i])->getWorldLocation();
		cog = cog + vec3( pos.toSbVec3f() );
	}
	cog = ( 1.0 / (int)mContacts.size() ) * cog;

	fprintf(stderr,"CoG: %f %f %f\n",cog.x(), cog.y(), cog.z());

	vec3 radius;
	double maxRadius = 0;
	for (int i=0; i<(int)mContacts.size(); i++) {
		pos = ((VirtualContact*)mContacts[i])->getWorldLocation();
		radius =  vec3( pos.toSbVec3f() ) - cog;
		if ( radius.len() > maxRadius) maxRadius = radius.len();
	}

	fprintf(stderr,"Max radius: %f\n",maxRadius);

	for (int i=0; i<(int)mContacts.size(); i++) {
		((VirtualContact*)mContacts[i])->setCenter( position(cog.toSbVec3f()) );
		((VirtualContact*)mContacts[i])->setRadius(maxRadius);
	}
}


void ContactExaminerDlg::updateQualityButton_clicked()
{
	
	mHand->getGrasp()->setObject(NULL);
	mHand->getGrasp()->update();
	double q = mQual->evaluate();
	fprintf(stderr,"Quality: %f\n",q);
	if (q < 0) q = 0;
	QString qs;
	qs.setNum(q);
	qs.truncate(5);
	qualityLabel->setText(qs);
}


void ContactExaminerDlg::sampleButton_clicked()
{
	double min = -1.5;
	double max = 1.5;
	double step = 0.025;

	EigenGraspInterface* mGrasps = mHand->getEigenGrasps();
	int eSize = mGrasps->getSize();
	double *amps = new double[eSize];
	double *dofs = new double[mHand->getNumDOF()];

	int level = eSize-1;
	bool legal, done = false;
	for (int i=0; i<eSize; i++)
		amps[i] = min;

//	int iterCount = 0;
	FILE *fp = fopen("sampled_quality.txt","w");

	while (!done) {
		//do stuff here
		mGrasps->getDOF(amps,dofs);
		legal = mHand->checkDOFVals(dofs);
		mHand->forceDOFVals(dofs);

		//if (iterCount % 8 == 0) graspItGUI->getIVmgr()->getViewer()->render();
		//iterCount++;

		mHand->emitConfigChange();
		mHand->getGrasp()->update();
		double q = mQual->evaluate();
		if (q < 0) q=0;
		for (int i=0; i<eSize; i++)
			fprintf(fp,"%f,",amps[i]);
		fprintf(fp,"%f,%d\n",q,legal);
   
		level = eSize-1;
		amps[level] += step;
		while (!done && amps[level] > max) {
			//fprintf(stderr,"level %d %f\n",level,amps[level]);
			amps[level] = min;
			level--;
			if (level < 0) done = true;
   else amps[level] += step;
   fprintf(stderr,"level %d %f\n",level,amps[level]);
  }
 }

 delete [] amps;
 delete [] dofs;
 fclose(fp);
}

void ContactExaminerDlg::clearButton_clicked()
{
 fprintf(stderr,"Coming soon...\n");
}

void ContactExaminerDlg::destroy()
{
 mContacts.clear();
 if (mQual) delete mQual;
}


void ContactExaminerDlg::showWrenchesButton_clicked()
{

}

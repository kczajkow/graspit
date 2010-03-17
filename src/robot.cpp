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
// Author(s):  Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: robot.cpp,v 1.68.2.2 2009/04/27 14:33:14 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the robot class hierarchy
 */
#include <iomanip>
#include <QFile>
#include <QTextStream>

//needed just for the image of the Flock of Birds sensor and the approach direction
#include "SoArrow.h"
#include <Inventor/nodes/SoCube.h>

#include "bBox.h"
#include "mytools.h"
#include "matvecIO.h"
#include "robot.h"
#include "joint.h"
#include "dynJoint.h"
#include "world.h"
#include "grasp.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "dynamics.h"
#include "humanHand.h"
#include "gloveInterface.h"
#include "eigenGrasp.h"
#include "matrix.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define PROF_ENABLED
#include "profiling.h"

PROF_DECLARE(MOVE_DOF);

#define AUTO_GRASP_TIME_STEP 0.01

/*! Removes the base and mountpiece from the world, and deletes the
	kinematic chains and DOFs.  If this robot is connected to a parent
	robot, it detaches itself.
*/
Robot::~Robot()
{
	for (int i=0;i<numChains;i++) {
		if (chainVec[i]) delete chainVec[i];
	}
	for (int i=0;i<numDOF;i++) {
		if (dofVec[i]) delete dofVec[i];
	}
	if (base) myWorld->destroyElement(base);  
	if (mountPiece) myWorld->destroyElement(mountPiece);
	if (parent) parent->detachRobot(this);
	if (mGloveInterface) delete mGloveInterface;
	if (mEigenGrasps) delete mEigenGrasps;

	std::cout << "Deleted robot: " << name() <<std::endl;
}

/*! Given a filename, it opens the file and then calls the private
	loadFromStream to parse it and load all the information provided.
	The filename is expected to be an absolute path. The robot will 
	make it relative to the GRASPIT root and save the result as its
	internal filename.
*/
int
Robot::load(QString filename)
{
	//the name is the filename without a path and without extension
	setName(filename.section('/',-1).section('.',0,0));
	//the filename is saved relative to the Graspit root
	myFilename = relativePath(filename, getenv("GRASPIT"));
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		DBGA("could not open robot configuration file" << filename.latin1());
		return FAILURE;
	}
	QString root = filename.section('/',0,-2,QString::SectionIncludeTrailingSep);
	QTextStream stream(&file);
	int result = loadFromStream(stream, root);
	if (result == FAILURE) {
		QTWARNING("Failed to load robot from file");
	}
	return result;
}

/*! Parses the \a stream for all the information that populates this class.
	All filenames specified in \a stream for loading various other bits
	of information are assumed to be relative to \a rootPath
*/
int
Robot::loadFromStream(QTextStream &stream, QString rootPath)
{
	// read and ignore the classType 
	QString line;  
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for class type");
		return FAILURE;
	}

	// read and load the base; automatically placed at origin
	DBGA("Creating base...\n");  
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for base");
		return FAILURE;
	}
	base = new Link(this,-1,-1,myWorld,(QString(name())+"Base").latin1());
	QString ivdir = rootPath + "iv/";
	if (!base  || base->load(ivdir+line.section(' ',0,0))==FAILURE) {
		if (base) delete base; 
		base = NULL;
		DBGA("Failed to load base");
		return FAILURE;
	}
	base->addToIvc();

	//init my IVRoot and add the base
	IVRoot = new SoSeparator;
	IVRoot->addChild(base->getIVRoot());

	//read number of DOFs and allocate them
	if (!nextValidLine(&stream, &line)){
		DBGA("Unexpected end of file looking for number of DOFs");
		return FAILURE;
	}
	bool ok;
	numDOF = line.toInt(&ok);
	if (!ok || numDOF < 1) {
		DBGA("Wrong number of DOFs specified: " << line.latin1());
		return FAILURE;
	}
	DBGA("Setting up " << numDOF << " degrees of freedom...");
	dofVec.resize(numDOF, NULL);

	//read each dof information
	for (int f=0;f<numDOF;f++) {
		if (!nextValidLine(&stream, &line)) {
			DBGA("Unexpected end of file looking for DOF " << f);
			return FAILURE;
		}
		QTextStream strStream(&line,QIODevice::ReadOnly);
		char dofType;
		strStream >> dofType;
		switch(dofType) {
			case 'r':
				dofVec[f] = new RigidDOF();
				break;
			case 'b':
				dofVec[f] = new BreakAwayDOF();
				break;
			case 'c':
				dofVec[f] = new CompliantDOF();
				break;
			default:
				DBGA("Unknown DOF Type requested: " << dofType << " for DOF " << f);
				return FAILURE;
		}
		dofVec[f]->dofNum = f;
		if (!dofVec[f]->readParametersFromStream(strStream)) {
			DBGA("Failed to read DOF " << f << ": " << line.latin1());
			return FAILURE;
		}
	}

	//read number of chains and allocate them
	if (!nextValidLine(&stream, &line)) {
		DBGA("Unexpected end of file looking for number of chains");
		return FAILURE;
	}
	numChains = line.toInt(&ok);
	if (!ok || numChains < 1) {
		DBGA("Wrong number of chains: " << line.latin1()); 
		return FAILURE;
	}
	DBGA("Creating " << numChains << " kinematic chains (fingers)...");
	chainVec.resize(numChains, NULL);

	//ask each chain to read itself from the file
	numJoints = 0;
	for (int f=0; f<numChains; f++) {
		fprintf(stderr, "Chain %d: ",f);
		chainVec[f] = new KinematicChain(this,f, numJoints);
		if (chainVec[f]->initChain(stream,ivdir)==FAILURE) {
			DBGA("Failed to read chain " << f);
			return FAILURE;
		}
		numJoints += chainVec[f]->getNumJoints();
	}

	//set up DOFs before setting up EigenGrasps
	std::list<Joint *>jointList;
	for (int d=0; d<numDOF; d++) {
		jointList.clear();
		for (int f=0; f<numChains; f++) {
			for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
				if (chainVec[f]->getJoint(j)->getDOFNum() == d) {
					jointList.push_back(chainVec[f]->getJoint(j));
				}
			}
		}
		dofVec[d]->initDOF(this,jointList);
	}

	//reset the dynamics: fix the base and set desired dof's to current values
	getBase()->fix();
	for (int i=0; i<getNumDOF(); i++) {
		getDOF(i)->setDesiredPos( getDOF(i)->getVal() );
	}

	//------------------from here on we are no longer reading the file sequentially
	
	//load approach direction
	approachTran = transf::IDENTITY;
	if (loadApproachDirection(&stream)==FAILURE) {
		DBGA("Failed to read approach direction");
	}
	addApproachGeometry();

	//load EigenGrasps
	stream.seek(0);
	mEigenGrasps = new EigenGraspInterface(this);
	bool eigenLoaded = false;
	if( findString(&stream,"EigenGrasps")) {
		stream >> line;
		QString eigenFile = rootPath + line;
		if (loadEigenData(eigenFile)==SUCCESS) {
			DBGA("Using eigengrasps from file: " << line.latin1());
			eigenLoaded = true;
		}
	}
	if (!eigenLoaded) {
		mEigenGrasps->setTrivial();
		DBGA("Using identity eigengrasps");
	}

	//load pre-defined, "virtual" contact locations
	stream.seek(0);
	if ( findString(&stream,"VirtualContacts")) {
		stream >> line;
		QString contactFile = rootPath + line;
		if (loadContactData(contactFile)==SUCCESS) {
			DBGA("Loaded virtual contacts from file " << line.latin1());
			showVirtualContacts(true);
		} else {
			DBGA("Failed to load virtual contacts from file " << line.latin1());
		}
	}
	
	//load CyberGlove information
	stream.seek(0);
	mUseCyberGlove = false;
	if (findString(&stream,"CyberGlove")) {
		mGloveInterface = new GloveInterface(this);
		stream >> line;
		QString calibFile = rootPath + line;	
		if (!mGloveInterface->loadCalibration(calibFile.latin1())) {
			DBGA("Failed to load glove calibration file " << calibFile.latin1());
			delete mGloveInterface; mGloveInterface = NULL;
			mUseCyberGlove = false;
		} else {
			DBGA("Cyberglove calibration loaded from " << calibFile.latin1());
			mUseCyberGlove = true;
		}
	}

	//load flock of birds information
	if ( loadFlockData(stream)==FAILURE ) {
		DBGA("There was a problem reading the Flock of Birds data in the configuration file");
	}
	if (mUsesFlock) {
		addFlockSensorGeometry();
	}
	return SUCCESS;
}

/*! Looks for the keyword "Approach" in the input stream. On the line following it,
	it expects the translation from base origin to the origin of the approach direction.
	On the line after that, is expects the approach direction itself. It sets up a transform
	that goes from base origin to the origin of the approach direction and aligns the z
	axis with the approach direction.
*/
int 
Robot::loadApproachDirection(QTextStream *stream)
{
	// Look for approach direction information
	stream->seek(0);
	if (!findString(stream,"Approach")) {
		return SUCCESS;
	} 
	QString line = stream->readLine();
	vec3 aPt = vec3(line.section(' ',0,0).toFloat(), 
					line.section(' ',1,1).toFloat(), 
					line.section(' ',2,2).toFloat());
	line = stream->readLine();
	vec3 zdir = vec3(line.section(' ',0,0).toFloat(), 
					 line.section(' ',1,1).toFloat(), 
					 line.section(' ',2,2).toFloat());
	zdir = normalise(zdir);
	
	vec3 xdir;
	if ( fabs(zdir % vec3(1,0,0)) > 0.9 ) xdir = vec3(0,1,0);
	else xdir = vec3(1,0,0);
	vec3 ydir = zdir * xdir;
	xdir = ydir * zdir;

	mat3 r(xdir, ydir, zdir);
	approachTran.set(r, aPt);
	return SUCCESS;
}

/*! Loads the eigengrasp information from the file \a filename. */
int
Robot::loadEigenData(QString filename)
{
	if ( !mEigenGrasps->readFromFile(filename.latin1()) ) {
		DBGA("Unable to load eigenGrasp file " << filename.latin1());
		return FAILURE;
	}
	QString name = filename.section('/',-1,-1);
	name = name.section('.',0,0);
	mEigenGrasps->setName(name);
	return SUCCESS;
}

/*!	Looks for the keyword "FlockOfBirds which denotes information about how this
	robot uses the Flock. On the line after it, it expects the number of the bird
	that this robot listens to. After that, it expects a tranform that indicates
	where the bird sensor is mounted on the palm.
*/
int
Robot::loadFlockData(QTextStream &stream)
{
	stream.seek(0);
	mUsesFlock = false;
	if (!findString(&stream,"FlockOfBirds")) {
		return SUCCESS;
	}

	QString line;
	stream >> line;
	mBirdNumber = line.section(" ",0,0).toInt();

	transf sensorTrans;
	if (readTransRotFromQTextStream(stream,sensorTrans) == FAILURE ) {
		DBGA("Unable to read base-origin-to-sensor-mount-transform");
		return FAILURE;
	}
	mFlockTran.identity();
	mFlockTran.setMount(sensorTrans);
	mUsesFlock = true;
	DBGA("Robot using Flock of Birds sensor " << mBirdNumber);
	return SUCCESS;
}

/*! Loads all the virtual contacts specified in the file \a filename
*/
int 
Robot::loadContactData(QString filename)
{
	FILE *fp = fopen(filename.latin1(), "r");
	if (!fp) {
		DBGA("Could not open contact file " << filename.latin1());
		return FAILURE;
	}

	char robotName[500];
	fscanf(fp,"%s",robotName); //yes, I know, can seg fault...

	if (getName() != robotName) {
		DBGA("Virtual Grasp defined for another robot!");
		fclose(fp);
		return FAILURE;
	}
	int numContacts;
	VirtualContact *newContact;
	fscanf(fp,"%d",&numContacts);
	for (int i=0; i<numContacts; i++) {
		newContact = new VirtualContact();
		newContact->readFromFile(fp);
		
		int f = newContact->getFingerNum();
		int l = newContact->getLinkNum();
		if ( f >= 0) {
			newContact->setBody( getChain(f)->getLink(l) );
			getChain(f)->getLink(l)->addVirtualContact( newContact );
		}
		else if (f == -1) {
			newContact->setBody( getBase() );
			getBase()->addVirtualContact(newContact);
		}
		else {
			fprintf(stderr,"Wrong finger number on contact!!!\n");
			delete newContact;
			continue;
		}
		newContact->computeWrenches(false,false);
	}
	fclose(fp);
	return SUCCESS;
}

/*! This robot becomes a "clone" of another robot, specified in \a original.
	This means that the new robot has its own kinematic structure, DOF's, etc
	but its links share the geometry of the original robot. See the clone 
	function in the Body class for details on how that works.
	
	This is generally used for multi-threading, if you want to do spred the 
	computations done on a robot to multiple cores. You then create multiple
	clones of your robot and pass one to each thread. See the threading 
	documentation in the collision detection classes for details.
*/
void
Robot::cloneFrom(Robot *original)
{
	myName = original->getName() + QString(" clone");

	//new robots have contact indicators disabled by default
	if (base) delete base;
	base = new Link(this,-1,-1,myWorld,(QString(name())+"Base").latin1());
	base->cloneFrom( original->getBase() );
	//base->initDynamics();
	IVRoot = new SoSeparator;
	IVRoot->addChild(base->getIVRoot());

	numDOF = original->getNumDOF();
	dofVec.resize(numDOF, NULL);
	for (int f=0; f<numDOF; f++) {
		switch(original->getDOF(f)->getType()) {
			case DOF::RIGID:
				dofVec[f] = new RigidDOF( (RigidDOF*)(original->getDOF(f)) );
				break;
			case DOF::BREAKAWAY:
				dofVec[f] = new BreakAwayDOF( (BreakAwayDOF*)(original->getDOF(f)) );
				break;
			case DOF::COMPLIANT:
				dofVec[f] = new CompliantDOF( (CompliantDOF*)(original->getDOF(f)) );
				break;
			default:
				DBGA("ERROR: Unknown DOF type in original!");
				assert(0);
		}
	}

	numChains = original->getNumChains();
	chainVec.resize(numChains, NULL);
	numJoints = 0;
	for (int f=0; f<numChains; f++) {
	    chainVec[f] = new KinematicChain(this,f,numJoints);
		chainVec[f]->cloneFrom( original->getChain(f) );
		numJoints += chainVec[f]->getNumJoints();
	}
	assert (numJoints == original->getNumJoints() );

	std::list<Joint *>jointList;
	for (int d=0; d<numDOF; d++) {
		jointList.clear();
		for (int f=0; f<numChains; f++) {
			for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
				if (chainVec[f]->getJoint(j)->getDOFNum() == d) {
					jointList.push_back(chainVec[f]->getJoint(j));
				}
			}
		}
		dofVec[d]->initDOF(this,jointList);
	}

	base->setTran(transf::IDENTITY);
	//careful: clones robots have never been tested with the dynamics engine
	getBase()->fix();
	for (int i=0; i<getNumDOF(); i++) {
	  getDOF(i)->setDesiredPos( getDOF(i)->getVal() );
	}

	setRenderGeometry( original->getRenderGeometry() );

	//check if original has an approach tran
	approachTran = original->getApproachTran();
	addApproachGeometry();

	//clone eigengrasp interface
	mEigenGrasps = new EigenGraspInterface(original->getEigenGrasps());

	//no glove interface
	mGloveInterface = NULL;
	this->mUseCyberGlove = false;
}

/*! Sets the transparency of all the links that make up this robot,
	as well as the base
*/
void
Robot::setTransparency(float t)
{
	base->setTransparency(t);
	for (int i=0; i<getNumChains(); i++) {
		for (int l=0; l<getChain(i)->getNumLinks(); l++) {
			getChain(i)->getLink(l)->setTransparency(t);
		}
	}
}

/*! Sets the name \a newName for the robot, as well as derived names of the form
	newName_chain#_link# for the links and newName_base for the base.
*/
void
Robot::setName(QString newName)
{
	WorldElement::setName(newName);
	for (int c=0; c<getNumChains(); c++) {
		for (int l=0; l<getChain(c)->getNumLinks(); l++){
			getChain(c)->getLink(l)->setName( newName + QString("_chain%1_link%2").arg(c).arg(l) );
		}
	}
	if (base) base->setName(newName + QString("_base"));
}

/*! Adds an arrow that shows the pre-defined approach direction for this
	robot. The arrow is rooted at the origin of the approach direction and
	points in the z direction of mApproachTran
*/
void
Robot::addApproachGeometry()
{	
	IVApproachRoot = new SoSeparator();
	SoTransform *t1 = new SoTransform();
	approachTran.toSoTransform(t1);
	IVApproachRoot->addChild(t1);

	SoArrow *arrow = new SoArrow;
	arrow->height = 14;
	arrow->cylRadius = (float)1.25;
	arrow->coneRadius = (float)2.6;
	arrow->coneHeight = (float)5.5;
	SoTransform *arrowTran = new SoTransform();
	arrowTran->rotation.setValue(SbVec3f(1,0,0),(float)(M_PI/2.0));
	IVApproachRoot->addChild(arrowTran);
	IVApproachRoot->addChild(arrow);
	getBase()->getIVRoot()->addChild(IVApproachRoot);	
}

/*! Adds a visual marker that shows where on the robot the Flock of Birds
	sensor is mounted. Uses the mounting information stored in the
	mFlockTran, which is usually pre-defined in the robot file
*/
void
Robot::addFlockSensorGeometry()
{
	IVFlockRoot = new SoSeparator();
	SoTransform *t = new SoTransform();
	mFlockTran.getMount().toSoTransform( t );
	IVFlockRoot->addChild(t);
	SoCube *cube = new SoCube();
	cube->width = 30;
	cube->height = 20;
	cube->depth = 4;
	IVFlockRoot->addChild(cube);
	SoCube *smallCube = new SoCube();
	smallCube->width = 10;
	smallCube->height = 20;
	smallCube->depth = 6;
	SoTransform *t2 = new SoTransform();
	t2->translation.setValue(10,0,-5);
	IVFlockRoot->addChild(t2);
	IVFlockRoot->addChild(smallCube);
	getBase()->getIVRoot()->addChild(IVFlockRoot);
}

/*! Sets the robot to use the trivial eigengrasps set, where each eigengrasp
	corresponds to a single DOF with an amplitude of 1.0. In this case, there
	is no difference between seting EG's and using DOF's directly.
*/
int
Robot::useIdentityEigenData()
{
	if (!mEigenGrasps->setTrivial()) {
		QTWARNING("Error setting Identity EigenGrasps");
		return FAILURE;
	}
	return SUCCESS;
}

/*! Tells the robot which CyberGlove the raw sensor data is coming from. The robot
	does not process raw data directly, just passes it to the mGloveInterface which
	hodsl calibration and DOF mapping between a raw Glove and this particular robot
*/
void Robot::setGlove(CyberGlove *glove)
{
	if (!mGloveInterface) {
		mUseCyberGlove = false;
		return;
	}
	mGloveInterface->setGlove(glove);
}

/*! Sets the values of the DOF's based on the information from the mGloveInterface,
	which has presumably processed a new batch of raw information from a
	real CyberGlove
*/
void
Robot::processCyberGlove()
{
	int i;
	double *desiredVals = new double[getNumDOF()];
	for (i=0; i<getNumDOF(); i++) {
		if ( mGloveInterface->isDOFControlled(i) ) {
			desiredVals[i] = mGloveInterface->getDOFValue(i);
			if ( desiredVals[i] < getDOF(i)->getMin() ) desiredVals[i] = getDOF(i)->getMin();
			if ( desiredVals[i] > getDOF(i)->getMax() ) desiredVals[i] = getDOF(i)->getMax();
		}
		else {
			desiredVals[i] = getDOF(i)->getVal();
		}
	}
	moveDOFToContacts(desiredVals, NULL, false);
	emitConfigChange();
	delete [] desiredVals;
}

/*!
  Given the body filename of a mountpiece, this will load it into the
  world and connect it to the base of this robot.
*/
Link *
Robot::importMountPiece(QString filename)
{
	QString mountName = QString(name())+"_mount";
	mountPiece = new Link(this,-1,-1,myWorld,mountName.latin1());
	if (mountPiece->load(filename)==FAILURE){
		delete mountPiece; mountPiece = NULL; return NULL;
	}
	mountPiece->addToIvc();
	IVRoot->addChild(mountPiece->getIVRoot());
	mountPiece->setTran(base->getTran());
 	base->setDynJoint(new FixedDynJoint(mountPiece,base));
	return mountPiece;
}

/*! Adds all the bodies associated with this robot (links, base, mountpiece,
	attached robots) to the given vector of bodies.
*/
void 
Robot::getBodyList(std::vector<Body*> *bodies)
{
	if (base) bodies->push_back(base);
	if (mountPiece) bodies->push_back(mountPiece);
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<chainVec[c]->getNumLinks(); l++) {
			bodies->push_back(chainVec[c]->getLink(l));
		}
	}
	for (int c=0; c<numChains; c++) {
		for (int i=0; i<chainVec[c]->getNumAttachedRobots(); i++) {
			chainVec[c]->getAttachedRobot(i)->getBodyList(bodies);
		}
	}
}

/*!
  Returns a vector of all links associated with this robot including
  all links in all chains, the base, and the mountpiece.
*/
void
Robot::getAllLinks(std::vector<DynamicBody *> &allLinkVec)
{
  int c,l,i;
  if (base) allLinkVec.push_back(base);
  if (mountPiece) allLinkVec.push_back(mountPiece);

  for (c=0;c<numChains;c++)
    for (l=0;l<chainVec[c]->getNumLinks();l++)
      allLinkVec.push_back(chainVec[c]->getLink(l));
  for (c=0;c<numChains;c++)
    for (i=0;i<chainVec[c]->getNumAttachedRobots();i++)
	  chainVec[c]->getAttachedRobot(i)->getAllLinks(allLinkVec);
}

/*!
  Returns a pointer to this robot and recursively, all child robots
  connected to this one
*/
void
Robot::getAllAttachedRobots(std::vector<Robot *> &robotVec)
{
	robotVec.push_back(this);
	for (int c=0;c<numChains;c++) {
		for (int i=0;i<chainVec[c]->getNumAttachedRobots();i++) {
			chainVec[c]->getAttachedRobot(i)->getAllAttachedRobots(robotVec);
		}
	}
}


/*!
  Given a pointer to a robot, the number of the chain on this robot to
  connect the new robot to, and the offset transform between the chain end
  and the base of the new robot, this will attach the new robot to this
  robot's chain end.
*/
void
Robot::attachRobot(Robot *r,int chainNum,const transf &offsetTr)
{
  r->parent = this;
  r->parentChainNum = chainNum;
  r->tranToParentEnd = offsetTr.inverse();
  chainVec[chainNum]->attachRobot(r,offsetTr);
}


/*!
  The detaches the robot pointed to by \a r from this robot so that they may
  move independently.
*/
void
Robot::detachRobot(Robot *r)
{
  DBGA("Detaching Robot " << r->getName().latin1() << " from " << getName().latin1());
  r->parent = NULL;
  chainVec[r->parentChainNum]->detachRobot(r);
}


/*!
  Given an array of DOF values equal in length to the number of DOFs in
  this robot, and the chain number for which the  kinematics should be
  computed, this will return a vector of transforms corresponding to
  the pose of each link in the chain.
*/
void
Robot::fwdKinematics(double* dofVals,std::vector<transf>& trVec,int chainNum)
{
	double *jointVals = new double[numJoints];
	getJointValues(jointVals);
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->accumulateMove(dofVals[d], jointVals, NULL);
	}
	chainVec[chainNum]->fwdKinematics(jointVals,trVec);
	delete [] jointVals;
}

/*!
  Unfortunately this is not complete.  Given a transform for the end pose
  of a particular kinematic chain, this will compute the DOF values
  corresponding to that pose.  It will use an iterative approximation
  technique, but I haven't had time to code this.  This routine is
  overriden for the Puma560 which has an analytical solution.
*/
int
Robot::invKinematics(const transf&, double* dofVals, int chainNum)
{
  // deals with compiler warnings
  dofVals = NULL;
  chainNum = 0;
  return FAILURE;
}


/*!
  This is an internal method called by contactsPreventMotion.  Given a
  motion transform for an entire robot that is defined with respect to the
  base frame of the robot, it recursively checks all of the
  kinematic chains of the robot and any robots connected to them to
  determine if any contacts will prevent the motion.
*/
bool
Robot::simpleContactsPreventMotion(const transf& motion) const
{
	transf linkMotion;
	transf baseToLink;

	for (int i=0;i<numChains;i++) {
		for (int j=0; j<chainVec[i]->getNumLinks();j++) {
			if (chainVec[i]->getLink(j)->getNumContacts()) {
				baseToLink = chainVec[i]->getLink(j)->getTran() * base->getTran().inverse();
				linkMotion = baseToLink * motion * baseToLink.inverse();
				if (chainVec[i]->getLink(j)->externalContactsPreventMotion(linkMotion))
					return true;
			}
		}
		// recursively check the motion for any robots attached to this chain
		for (int j=0;j<chainVec[i]->getNumAttachedRobots();j++) {
			baseToLink = chainVec[i]->getAttachedRobot(j)->getBase()->getTran() * base->getTran().inverse();
			linkMotion = baseToLink * motion * baseToLink.inverse();
			if (chainVec[i]->getAttachedRobot(j)->simpleContactsPreventMotion(linkMotion))
				return true;
		}
	}
    return false;
}


/*!
  Examines all of the contacts on every link of this robot, and each
  child robot connected to this one, to determine if the desired motion
  can be performed.  It also performs the inverse kinematics of the
  parent robot to see if any of those link motions will be prevented by
  contacts on those links.  The motion is expressed with respect to the
  base frame of the robot.  This is only used when dynamics are off.
*/
bool
Robot::contactsPreventMotion(const transf& motion) const
{
	int l;
	transf linkMotion;

	// check if we can move the base link
	if (base->externalContactsPreventMotion(motion)) return true;
	if (mountPiece && mountPiece->contactsPreventMotion(motion)) return true;

	// check if we can move all the kinematic chains and the robots connected to them
	if (simpleContactsPreventMotion(motion)) return true;

	// if this robot is connected to another before it, try to do inv kinematics
	// and check if contacts on any of those links will prevent this motion
	if (parent) {
		transf newTran = tranToParentEnd*motion*base->getTran() * parent->getBase()->getTran().inverse();
		KinematicChain *chain = parent->getChain(parentChainNum);
		std::vector<transf> parentTrVec(chain->getNumLinks());    
		double *dofVals = new double[parent->getNumDOF()];

		if (parent->invKinematics(newTran,dofVals,parentChainNum)==FAILURE){
			delete [] dofVals;
			return true;
		}
    
		parent->fwdKinematics(dofVals,parentTrVec,parentChainNum);
		delete [] dofVals;

		for (l=0;l<chain->getNumLinks();l++) {      
			if (chain->getLink(l)->getNumContacts()) {
				linkMotion = parentTrVec[l] * chain->getLink(l)->getTran().inverse();
				if (chain->getLink(l)->contactsPreventMotion(linkMotion)) {
					return true;
				}
			}
		} 
	}
    return false;
}

/*! Breaks all the contacts formed on the links or the base of this robot */
void
Robot::breakContacts()
{
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			getChain(f)->getLink(l)->breakContacts();
		}
	}
	base->breakContacts();
	if (mountPiece) mountPiece->breakContacts();
}


/*! This can be used to disable / enable the automatic render requests from this robot. 
	If this is disabled, changes to the robot pose or posture should not trigger an
	automatic render request through Coin. The robot however is still part of the scene 
	graph, so whenever a render request comes from someplace else, the robot is rendered
	in its most current state. See also the option in the World class to completely
	remove a robot from the scene graph.
*/
void
Robot::setRenderGeometry(bool s)
{
	mRenderGeometry = s;
	if (parent) parent->setRenderGeometry(s);
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<chainVec[c]->getNumAttachedRobots(); j++) {
			chainVec[c]->getAttachedRobot(j)->setRenderGeometry(s);
		}
	}
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			getChain(f)->getLink(l)->setRenderGeometry(s);
		}
	}
	base->setRenderGeometry(s);
	if (mountPiece) mountPiece->setRenderGeometry(s);
}

/*!
  This attempt to set the pose of the robot base to tr.  It does not check
  for collisions, but it will check the inverse kinematics of a parent
  robot (if it exists) to determine whether the move is valid.  If it is
  valid the DOF's of the parent are set, and this robot and all of its
  children are moved.
*/
int
Robot::setTran(const transf& tr)
{

  if (parent) {
    double *dofVals = new double[parent->getNumDOF()];
    transf parentBaseToEnd =
      tranToParentEnd*tr*parent->getBase()->getTran().inverse();

    if (parent->invKinematics(parentBaseToEnd,dofVals,parentChainNum)==FAILURE) {
	  delete [] dofVals;
      return FAILURE;
    }

#ifdef GRASPITDBG
    printf("Setting DOF vals: ");
    for (int i=0;i<parent->getNumDOF();i++)
      printf("%lf ",dofVals[i]);
    printf("\n");
#endif

    parent->forceDOFVals(dofVals);
	delete [] dofVals;
  }
  else {
    simpleSetTran(tr);  
  }
  return SUCCESS;
}

/*! Returns the total number of links of this robot, including the palm and
	the mount piece (if any).
*/
int
Robot::getNumLinks() const
{
	int links=0;
	for (int k=0; k<getNumChains(); k++) {
		links += chainVec[k]->getNumLinks();
	}
	links++; //palm
	if (mountPiece) links++;
	return links;
}

/*! Returns the total number of contacts on this robot, including all links
	and the palm.
*/
int
Robot::getNumContacts(Body *body)
{
	int c = getBase()->getNumContacts(body);
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			c += getChain(f)->getLink(l)->getNumContacts(body);
		}
	}
	return c;
}

/*! Returns the total number of virtual contacts on this robot, including all 
	links and the palm.
*/
int
Robot::getNumVirtualContacts()
{
	int c = getBase()->getNumVirtualContacts();
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			c += getChain(f)->getLink(l)->getNumVirtualContacts();
		}
	}
	return c;
}

/*! Shows or hides the virtual contact on this robot (which can be shown
	as thin red cylinders). They are usually hidden so they don't trigger
	a redraw when not wanted, although we now have a better mechanism 
	for that using setRenderGeometry on the entire robot 
*/
void
Robot::showVirtualContacts(bool on)
{
	int s; bool b;
	if (on) s = 1;
	else s = 2;
	b = getBase()->frictionConesShown();
	getBase()->showFrictionCones(b, s);
	for (int f=0; f<getNumChains(); f++) {
		for (int l=0; l<getChain(f)->getNumLinks(); l++) {
			b = getChain(f)->getLink(l)->frictionConesShown();
			getChain(f)->getLink(l)->showFrictionCones( b, s );
		}
	}
}

//----------------------------------save and load state-------------------------

/*! Saves the current state of the robot, which can be restored later. 
	Overwrites any previously saved state. Maybe at some point we will 
	unify this with the stack of dynamic states which can hold any number
	of states for a body.
*/
void 
Robot::saveState()
{
	savedTran = getTran();
	savedDOF.clear();
	QTextStream stream(&savedDOF,QIODevice::ReadWrite);
	writeDOFVals(stream);
	savedState = true;
}

/*! Restores the previously saved state of this robot. If no state has been
	saved since the last restore, is prints out a warning, but does not
	die in pain.
*/
void
Robot::restoreState()
{
	if (!savedState) {
		DBGA("Warning: hand state not saved!");
		if (savedDOF.size()==0) {
			DBGA("Saved DOF data absent; can not restore");
			return;
		}
	}
	savedState = false;
	setTran( savedTran );
	QTextStream stream(&savedDOF,QIODevice::ReadOnly);
	readDOFVals(stream);
}

/*! Reads DOF values from a text stream (usually from a saved world file or an 
	internally saved state) and sets them
*/
QTextStream&
Robot::readDOFVals(QTextStream &is)
{
	for (int d=0; d<numDOF; d++) {
		if (!dofVec[d]->readFromStream(is)) {
			DBGA("Failed to read DOF " << d << " from stream");
			return is;
		}
	}

	// This is a somewhat contorted way of doing things, but this is the only mechanism available
	// for actually moving the DOFs 
	double *jointVals = new double[numJoints];
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->getJointValues(jointVals);
	}
	setJointValuesAndUpdate(jointVals);
	delete [] jointVals;
	return is;
}

/*! Writes DOF values to a text stream (usually a world save file or the 
	internally saved state)
*/
QTextStream&
Robot::writeDOFVals(QTextStream &os)
{
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->writeToStream(os);
		os << " ";
	}
	return os;
}

//---------------------------------- STATIC MOVEMENT FUNCTIONS--------


/*! This is one of the core function used for moving the DOF's of a robot in statics.
	Its purpose is to solve a collision: given an initial set of joint values, which
	should be collision-free, and a set of final joint values that result in a collision,
	it will interpolate between the two to find the initial moment of contact (where 
	the bodies are not inter-penetrating, but are separated by less then the contat
	threshold.
	
	In order to be efficient, it only checks the bodies that are given in the 
	\a colReport. If other bodies are also colliding at any point during the interpolation,
	this function will never know about it.
	
	Returns 0 if the interpolation fails (usually because the starting configuration
	is also in collision, or 1 if its succeeds. In case of success, \a
	interpolationTime will hold the interpolation coefficient that resulted in the
	original contact.
*/	
int
Robot::interpolateJoints(double *initialVals, double *finalVals, 
						 CollisionReport *colReport, double *interpolationTime)
{
	double *currentVals = new double[ getNumJoints() ];
	double minDist,t = 0.0,deltat = 1.0;
	while (deltat > 1.0e-20 && t >= 0) {

		DBGP("DOF joint interpolation cycle")
		deltat *= 0.5;
		for (int j=0; j<getNumJoints(); j++) {
			currentVals[j] = t*finalVals[j] + (1.0-t)*initialVals[j];
		}
		setJointValuesAndUpdate(currentVals);
 
		minDist = 1.0e10;
		for (int i=0; i<(int)colReport->size(); i++) {
			minDist = MIN(minDist,myWorld->getDist( (*colReport)[i].first, (*colReport)[i].second));
			if ( minDist <= 0 ) {
				t -= deltat;
				break;
			}
		}
		if (minDist > 0) {
			if (minDist < 0.5 * Contact::THRESHOLD) break;
			t += deltat;
		}
	}

	int retVal;
	if (deltat < 1.0e-20 || t < 0) {
#ifdef GRASPITDBG
		std::cerr << "t: " << t << "  d: " << deltat << std::endl;
		std::cerr << "I am " << getName().latin1() << std::endl;
		for (int i=0; i<(int)colReport->size(); i++) {
			std::cerr << (*colReport)[i].first->getName().latin1()<<" -- " 
				<< (*colReport)[i].second->getName().latin1() << std::endl;
		}
		std::cerr << "min dist: " << minDist << std::endl << std::endl;
#endif
		DBGA("Robot joint interpolation error");
		retVal = 0;
	} else {
		DBGP("Interpolation to t=" << t << " (deltat=" << deltat << ")");
		retVal = 1;
	}
	delete [] currentVals;
	*interpolationTime = t;
	return retVal;
}

/*! A utility function for the main static DOF movement functions. Given a link
	that is stopped (presumably due to some contact) it will mark all the joints
	that affect that link as stopped. This is done by setting the a bit in the 
	entry that correspinds to a given joint in the vector \a stoppedJoints. 
	\a stoppedJoints is assumed to be large enough for all the joints in the robot.
*/
void Robot::stopJointsFromLink(Link *link, double *desiredJointVals, int *stoppedJoints)
{
	if (link->getChainNum()<0) return; //it's the base
	KinematicChain *chain = chainVec[link->getChainNum()];
	for (int j=chain->getLastJoint( link->getLinkNum() ); j>=0; j--) {
		int jnum = chain->getJoint(j)->getNum();
		if (desiredJointVals[jnum] > chain->getJoint(j)->getVal()) {
			//stopped in the positive direction
			stoppedJoints[jnum] |= 1;
		} else if (desiredJointVals[jnum] < chain->getJoint(j)->getVal()){
			//stopped in the negative direction
			stoppedJoints[jnum] |= 2;
		}
	}
}

/*! One of the main functions for moving DOF's in statics. Given a set of \a
	desiredDofVals, this will ask the DOF's what values should be given to the
	joints. The vector \a stoppedJoints contains information about which joints
	have been stopped due to contacts. On exit, \a jointVals will contain the 
	needed joint values, and \a actualDofVals will contain the DOF values that
	were actually set (which might be different from \a desireDofVals because
	contacts might prevent the desired motion.
	
	The main reason for this implementation is that different types of DOF's 
	react different to contacts and implement coupling in their own differnt
	way.
	
	Returns \a true if at least on the joints of the robot is still moving, or
	\false if contacts prevent all motion, limits have been reached and no more
	movement is possible.
*/
bool
Robot::getJointValuesFromDOF(const double *desiredDofVals, double *actualDofVals, 
							 double *jointVals, int *stoppedJoints)
{
	bool done,moving;
	std::vector<transf> newLinkTran;
	DBGP("Getting joint movement from DOFs");
	do {
		moving = false; done = true;
		getJointValues(jointVals);
		//compute the aggregate move for all DOF's
		for (int d=0;d<numDOF;d++){
			//this check is now done by each DOF independently
			//if ( fabs(dofVals[d] - dofVec[d]->getVal()) < 1.0e-5) continue;
			if (dofVec[d]->accumulateMove(desiredDofVals[d], jointVals,stoppedJoints)) {
				moving = true;
				actualDofVals[d] = desiredDofVals[d];
				DBGP("DOF " << d << " is moving");
			} else {
				actualDofVals[d] = dofVec[d]->getVal();
			}
		}
		if (!moving) {
			DBGP("No DOF movement; done.");
			break;
		}
		//see if motion is allowed by existing contacts
		for (int c = 0; c < getNumChains(); c++) {
			KinematicChain *chain = getChain(c);
			newLinkTran.resize(chain->getNumLinks(), transf::IDENTITY);
			chain->infinitesimalMotion(jointVals, newLinkTran);
			for (int l=0; l<chain->getNumLinks(); l++){
				Link *link = chain->getLink(l);
				transf motion = newLinkTran[l];
				if ( link->contactsPreventMotion(motion) ) {
					DBGP("Chain " << link->getChainNum() << " link " << link->getLinkNum() << " is blocked.");
					//we stop all joints that are in the same chain before the stopped link
					stopJointsFromLink(link, jointVals, stoppedJoints);
					done = false;
					//once proximal links in a chain have been stopped, let's process distal links again
					//before making a decision on them, as their movement will be different
					break;
				}
			}
		}
		if (done) {
			DBGP("All movement OK; done.");
		}
	} while (!done);
	return moving;
}

/*! The core of the robot DOF movement in statics. Given a set of \a desiredVals for the
	DOF's, it will set the DOF's to that value. If the result is free of contact, we are
	done. If the result is in collision, it will interpolate to find the exact moment
	of contact and set the DOF's at that value. \a stoppedJoints marks any joints that
	can not move (presumably due to some contact discovered earlier). Returns the number
	of collisions found (and resolved) in \a numCols.
	
	After completing the move, it will also mark all the new contacts that have appeared
	due to the move. It will also mark as stopped the joints of the links that are now in 
	contact, by setting the appropriate bits in \a stoppedJoints.
	
	In theory, this function should always leave the robot in a legal state, with no
	inter-penetrations. 
	
	Returns \a true if the move had been performed successfully. Returns \a false if either
	no motion was performed because all dofs are already at the desired values, or contacts
	prevent all motion. Also returns \a false if there is an error in the interpolation.
	
	A final note of warning: this rather involved way of doing this was dictated by multiple
	problems: always avoid leaving the robot in an illegal state; allow different dof's to
	react to stopped joints in their own way; don't compute contacts more often then you
	have to (it's expensive); make sure the dof's move together, and not one by one; handle
	the case where the collision detection engine finds a collision, but fails to detect a 
	contact after it's solved (happens very rarely, but it does); etc. In general, this whole
	process is more complicated than it appears at first. We would love a general and robust
	solution to this, but tread carefully here.
*/
bool
Robot::jumpDOFToContact(double *desiredVals, int *stoppedJoints, int *numCols)
{
	CollisionReport colReport, lateContacts;

	//get new joint values and see which dof's are moving
	double *newDofVals = new double[ getNumDOF() ];
	double *initialDofVals = new double [ getNumDOF() ];
	getDOFVals(initialDofVals);

	double *newJointVals = new double[ getNumJoints() ];
	double *initialJointVals = new double[ getNumJoints() ];
	getJointValues(initialJointVals);

	if (!getJointValuesFromDOF(desiredVals, newDofVals, newJointVals, stoppedJoints)) {
		DBGP("No movement of DOFs; forceDOFTo returning false");
		if (numCols) *numCols = 0;
		delete [] initialJointVals;	delete [] newJointVals;
		delete [] initialDofVals; delete [] newDofVals;
		return false;
	}
	
	//compute list of links that have actually moved
	//question: should the dof's compute this?
	std::vector<Body*> interestList;
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<chainVec[c]->getNumLinks(); l++) {
			int j = chainVec[c]->getLastJoint(l);
			if ( !stoppedJoints[ chainVec[c]->getJoint(j)->getNum() ] ) {
				interestList.push_back( chainVec[c]->getLink(l) );
			}
		}
	}

	setJointValuesAndUpdate(newJointVals);
	bool interpolationError = false;
	double interpolationTime;
	while (1) {
		myWorld->getCollisionReport(&colReport, &interestList);
		//if we are free of collision, we are done
		if (colReport.empty()) break;
		//if not, we need to interpolate
		//goal is to interpolate to current joint values
		getJointValues(newJointVals);
		if (!interpolateJoints(initialJointVals, newJointVals, &colReport, &interpolationTime)) {
			DBGA("Interpolation failed!");
			interpolationError = true;
			break;
		}
		//save the list of contacting bodies
		lateContacts.clear();
		for (int i=0;i<(int)colReport.size();i++) {
			lateContacts.push_back( colReport[i] );
			DBGP("Contact: " << colReport[i].first->getName().latin1() << "--" << 
				  colReport[i].second->getName().latin1());
		}
		for (int d=0; d<numDOF; d++) {
			DBGP("Dof " << d << "initial " << initialDofVals[d] << " new " << newDofVals[d]);
			newDofVals[d] =     newDofVals[d] * interpolationTime + 
							initialDofVals[d] * ( 1.0 - interpolationTime);
			DBGP("Interpolated: " << newDofVals[d]);
		}
	}

	if (!interpolationError) {
		DBGP("ForceDOFTo done");
		myWorld->findContacts(lateContacts);
		for (int i=0; i<(int)lateContacts.size(); i++) {
			if (lateContacts[i].first->getOwner()==this) {
				stopJointsFromLink( (Link*)lateContacts[i].first, newJointVals, stoppedJoints);
			}
			if (lateContacts[i].second->getOwner()==this) {
				stopJointsFromLink( (Link*)lateContacts[i].second, newJointVals, stoppedJoints);
			}
		}
		updateDofVals(newDofVals);
		if (numCols) *numCols = (int)lateContacts.size();
	}

	delete [] initialDofVals; delete [] newDofVals;
	delete [] initialJointVals; delete [] newJointVals;

	if (!interpolationError) return true;
	else return false;
}


/*! This is the only interface for the user to perform robot DOF movement in statics.
	The desired dof vals are specified in \a desiredVals, which should be of size
	equal to the number of dofs of this robot.
	
	The most important aspect is that the motion can be broken down in small steps, to
	make sure that no collisions are missed along the way by "jumping through" an obstacle.
	The other important aspect is that, when a contact is found and a DOF is stopped,
	the other DOF's can continue to move.
	
	\a desiredSteps holds the size of the desired steps for each DOF. We usually use 10 
	degrees	for rotational DOFs and 50*Contact::THRESHOLD for translational DOF's. You can 
	use WorldElement::ONE_STEP if no stepping is to be performed. If \a desiredSteps is 
	NULL it has the same effect as setting all entried to ONE_STEP.
	
	If \a stopAtContact is true, all movement ends as soon as the first contact is detected.
	If not, movement proceeds until all DOF's have either reached the desired value, reached
	their limit, or have been stopped due to contact.
	
	Works by breaking down the motion in small time steps and uses the internal forceDOFTo 
	function repeatedly for each step.Returns true if the joints were moved; if no movement 
	was possible from the beginning, it returns false.
*/

bool 
Robot::moveDOFToContacts(double *desiredVals, double *desiredSteps, bool stopAtContact, bool renderIt)
{
//	PROF_RESET_ALL;
//	PROF_START_TIMER(MOVE_DOF);
	PROF_TIMER_FUNC(MOVE_DOF);

	int i,d;
	CollisionReport result;

	double *stepSize= new double[numDOF];
	double *currVals = new double[numDOF];
	double *newVals = new double[numDOF];
	
	for (i=0;i<numDOF;i++) {
		DBGP("from " << dofVec[i]->getVal() << " to " << desiredVals[i] << " in " << desiredSteps[i] << " steps");
		if (!desiredSteps || desiredSteps[i] == WorldElement::ONE_STEP ) {
			stepSize[i] = desiredVals[i] - dofVec[i]->getVal();
		} else if (desiredSteps[i]!=0.0) {
			stepSize[i] = desiredSteps[i];
		} else {
			desiredVals[i] = dofVec[i]->getVal();
		}
	}

	//this vector will keep tracked of the joints that have been stopped
	//at previous time steps due to contacts.
	int *stoppedJoints = new int[numJoints];
	for (int j=0; j<numJoints; j++) {
		stoppedJoints[j] = 0;
	}

	//loop until termination conditions are met
	int numCols,itercount = 0;
	bool moved = false;
	do {
		itercount++;
		DBGP("Move to contact cycle")
		getDOFVals(currVals);
		for (d=0;d<numDOF;d++) {
			newVals[d] = currVals[d] + stepSize[d];
			if (stepSize[d] > 0 && newVals[d] > desiredVals[d])
				newVals[d] = desiredVals[d];
			else if (stepSize[d] < 0 && newVals[d] < desiredVals[d])
				newVals[d] = desiredVals[d];
		}
		//perform one more step. jumpDOFToContact will inform us if no more
		//movement is done (or possible) and we can exit.
		if (!jumpDOFToContact(newVals, stoppedJoints, &numCols)){
			DBGP("ForceDOFTo reports no movement is possible");
			break;
		}
		moved = true;
		bool stopRequest = false;
		//inform whoever is listening a step has been performed
		emit moveDOFStepTaken(numCols, stopRequest);
		if (stopRequest) {
			DBGP("Receiver of movement signal requests early exit");
			break;
		}
		//check if we stop at first contact
		if (stopAtContact && numCols != 0) {
			DBGP("Stopping at first contact");
			break;
		}
		if (itercount > 500) {
			DBGA("MoveDOF failsafe hit");
			break;
		}
		if (renderIt && (itercount%25==0) && graspItGUI && graspItGUI->getIVmgr()->getWorld()==myWorld) {
			graspItGUI->getIVmgr()->getViewer()->render();
		}
	} while (1);
		
	//myWorld->resetDynamicWrenches();
	//accumulateQuasistaticForces(false);
	//graspItGUI->getIVmgr()->drawDynamicForces();
	//graspItGUI->getIVmgr()->drawUnbalancedForces();
//	PROF_STOP_TIMER(MOVE_DOF);
//	PROF_PRINT_ALL;

	delete [] currVals;
	delete [] newVals;
	delete [] stepSize;
	delete [] stoppedJoints;
	return moved;
}

/*! This function checks if the path to the desired vals is legal. Breaks 
	down motion into steps and does each step. As soon as any collision is 
	detected, it returns false. Duplicates a lot of the functionality of 
	moveDOFToContacts. This is not a very good implementation and should
	probably be cleaned up in the future.
*/
bool
Robot::checkDOFPath(double *desiredVals, double desiredStep)
{
	int d;
	bool done, success = true;

	double *stepSize= new double[numDOF];
	double *currVals = new double[numDOF];
	double *newVals = new double[numDOF];

	for (d=0;d<numDOF;d++) {
		if (desiredStep == WorldElement::ONE_STEP )
			stepSize[d] = desiredVals[d] - dofVec[d]->getVal();
		else if ( desiredVals[d] >= dofVec[d]->getVal() )
			stepSize[d] = desiredStep;
		else stepSize[d] = - desiredStep;
	}

	do {
		DBGP("Move to contact cycle")
		getDOFVals(currVals);
		for (d=0;d<numDOF;d++) {
			newVals[d] = currVals[d] + stepSize[d];
			if (stepSize[d] > 0 && newVals[d] > desiredVals[d])
				newVals[d] = desiredVals[d];
			else if (stepSize[d] < 0 && newVals[d] < desiredVals[d])
				newVals[d] = desiredVals[d];
		}

		forceDOFVals(newVals);

		if ( !myWorld->noCollision() ) {
			success = false;
			break;
		}

		done = true;
		for (d=0;d<numDOF;d++) {
			if (newVals[d] != desiredVals[d]) done = false;
		}

	} while (!done);

	delete [] currVals;
	delete [] newVals;
	delete [] stepSize;
	return success;
}

//-----------------------------static joint torques computations------------------------

/*! Returns the static torques on all the joints of this robot. This is 
	relevant for underactuated compliant hands only, should be zero in all 
	other cases. See the CompliantDOF class for details.
	
	If \useDynamicDofForce is set, the static joint torques are computed using
	the dynamic force currently set by each DOF. If not, the minimum values for
	balancing the system are computed. This is somewhat hacky as this is done
	in statics, not dynamics, but we needed a way of computing static joint 
	torques for some pre-specified level of DOF force.
*/
Matrix 
Robot::staticJointTorques(bool useDynamicDofForce)
{
	std::vector<double> jointTorques(getNumJoints(), 0.0);
	for (int d=0; d<numDOF; d++) {
		double dofForce = -1;
		if (useDynamicDofForce) {
			dofForce = dofVec[d]->getForce();
		}
		dofVec[d]->computeStaticJointTorques(&jointTorques[0], dofForce);
	}
	return Matrix(&jointTorques[0], getNumJoints(), 1, true);
}

/*! Computes the quasistatic contact forces that balance the system in its
	current configuration. Only relevant for underactuated compliant hands;
	see CompliantDOF class for details.
	
	This is an obsolete handle for this, as it will look at all contact on the
	robot regardless of the objects they are on. A better way is to go through
	the corresponding function of the \a Grasp class (which is done in the
	overloaded version of this in the Hand class). In the future, a combination
	of both might be needed.
	
	If the \a useDynamicDofForce flag is set, the dof forces set by the dynamics 
	engine are used. Otherwise, the smallest forces that balance the system are 
	computed. In the case of rigid and fully actuated dofs, these are ususally 0.
	For coupled and compliant dofs, maybe not. 
	
	Return codes: 0 is success; >0 means unbalanced forces; 
	<0 means error in computation 
*/
int
Robot::accumulateQuasistaticForces(bool useDynamicDofForce)
{
	Matrix tau(staticJointTorques(useDynamicDofForce));
	int retVal = 0;
	for (int c=0; c<numChains; c++) {
		retVal = chainVec[c]->accumulateStaticLinkForcesWithJacobian(tau);
		if (retVal) break;
	}
	return retVal;
}

//---------------------- miscellaneous functions ---------------------------

/*! Tells us how far along the pre-specified approach distance a certain object is.
	Since the approach direction might never intersect the given body, it also
	needs a cap telling is how far to look. This cap is \a maxDist. Therefore,
	a return value of \a maxDist might mean that the object is never hit by moving
	along the approach direction.
*/
double
Robot::getApproachDistance(Body *object, double maxDist)
{
	position p0 = position(0,0,0) * getApproachTran() * getTran();
	position p = p0;
	vec3 app = vec3(0,0,1) * getApproachTran() * getTran();
	bool done = false;
	double totalDist = 0;
	vec3 direction;
	int loops = 0;
	while (!done) {
		loops++;
		//compute shortest distance between current point and object;
		direction = myWorld->pointDistanceToBody(p, object);

		//current closest point on the object
		position pc = p; pc+=direction;
		//and its direction rel. to approach direction
		if ( normalise(pc-p0) % app > 0.86 ) {
			done = true;
		}
		//advance along approach direction
		double d = direction.len();
		totalDist += d;
		p += d * app;
		if (totalDist > maxDist) done = true;
		if (loops > 10) {
			done = true;
			totalDist = maxDist+1;
			DBGA("Force exit from gettAppDist");
		}
	}
	//if (totalDist > maxDist) {DBGA("Object far away in " << loops << " loop(s)");}
	//else {DBGA("Object at " << totalDist << " in " << loops << " loop(s)");}
	return totalDist;
}

//-----------------------DYNAMICS-------------------------------

/*! This is the main function for moving the robot DOF's in dynamics. All the
	user can do is set the desired dof vals. This function will compute a smooth 
	trajectory for each joint so that each has a smooth acceleration and velocity
	profile. The intermediate values become the setpoints for each
	joint controller.
	
	After that, we rely on the simulation world to run each dynamic time step and
	call the robot's dof controllers which will set dof dynamic forces based on
	the desired dof values. The next dynamic time step will then move the links
	based on the forces applied by the dof's.
	
	In practice, the dof controllers are tricky and I've never been able to make
	them very robust. They work reasonably well, but not in all cases.
*/
void
Robot::setDesiredDOFVals(double *dofVals)
{
  int i,j,d,numSteps;
  double timeNeeded;
  double t;
  double coeffs[5];    
  double tpow,q1,q0,qd0,qd1;
  double *traj;

  for (d=0;d<numDOF;d++) {
	if (dofVec[d]->getDefaultVelocity() == 0.0) continue;

    DBGP("DOF "<<d<<" trajectory");
    dofVec[d]->setDesiredPos(dofVals[d]);
    if (dofVec[d]->getVal() != dofVals[d]) {

	  q0 = dofVec[d]->getVal();
      q1 = dofVals[d];
      qd0 = 0.0;//dofVec[d]->getActualVelocity();
      qd1 = 0.0;

      timeNeeded = fabs(dofVals[d]-dofVec[d]->getVal()) / fabs( dofVec[d]->getDesiredVelocity() );

      //make this a whole number of timesteps
      numSteps = (int)ceil(timeNeeded/myWorld->getTimeStep());
      timeNeeded = numSteps*myWorld->getTimeStep();

      DBGP("numSteps: "<< numSteps << " time needed: " << timeNeeded);
      traj = new double[numSteps];

      for (i=0;i<numSteps;i++) {
		t = (double)i/(double)(numSteps-1);
		coeffs[4] = 6.0*(q1 - q0) - 3.0*(qd1+qd0)*timeNeeded;
		coeffs[3] = -15.0*(q1 - q0) + (8.0*qd0 + 7.0*qd1)*timeNeeded;
		coeffs[2] = 10.0*(q1 - q0) - (6.0*qd0 + 4.0*qd1)*timeNeeded;
		coeffs[1] = 0.0;
		coeffs[0] = qd0*timeNeeded; 
		traj[i] = q0;

		tpow = 1.0;
		for (j=0;j<5;j++) {
			tpow *= t;
			traj[i] += tpow*coeffs[j];
		}
		DBGP(i << " " << t << " " << traj[i]);
	  }
      dofVec[d]->setTrajectory(traj,numSteps);
      delete [] traj;
    }
  }
}

/*!
  Given a kinematic chain number and a vector of chain end poses, this
  computes the inverse kinematics for each pose, and computes a smooth
  trajectory for each DOF to reach each pose.
*/
void
Robot::setChainEndTrajectory(std::vector<transf>& traj,int chainNum)
{
  int i,j,numPts = traj.size();
  double *dofVals = new double[numDOF];
  bool *dofInChain = new bool[numDOF];
  
  if (numPts < 1 || chainNum < 0 || chainNum > numChains-1) return;
  for (j=0;j<numDOF;j++)
    dofInChain[j] = false;

  for (j=0;j<chainVec[chainNum]->getNumJoints();j++)
    dofInChain[chainVec[chainNum]->getJoint(j)->getDOFNum()] = true;

  invKinematics(traj[0],dofVals,chainNum);
  for (j=0;j<numDOF;j++)
    if (dofInChain[j]) dofVec[j]->setTrajectory(&dofVals[j],1);
			 
  for (i=1;i<numPts;i++) {
    invKinematics(traj[i],dofVals,chainNum);
    for (j=0;j<numDOF;j++)
      if (dofInChain[j]) dofVec[j]->addToTrajectory(&dofVals[j],1);
  }
  delete [] dofVals;
  delete [] dofInChain;
}


/*!
  Given a start and end pose, this creates a number of intermediate poses
  that linearly interpolate between the two in cartesian space.  The
  start and end velocities of the end effector default to 0 but can be
  changed.  If the time needed for the move is not set, the trajectory
  will have an average velocity equal to the \a defaultTranVel and
  \a defaultRotVel.  The number of poses generated is determined by the
  amount of time needed for the move divided by the default dyanmic time
  step length.
*/
void
Robot::generateCartesianTrajectory(const transf &startTr, const transf &endTr,
				   std::vector<transf> &traj,
				   double startVel, double endVel,
				   double timeNeeded)
{
  int i,j,numSteps;
  double t,tpow,dist,angle,coeffs[5];    
  vec3 newPos,axis;
  Quaternion newRot;
    
  (endTr.rotation() * startTr.rotation().inverse()).ToAngleAxis(angle,axis);
  
  if (timeNeeded <= 0.0) {
    if (defaultTranslVel == 0.0 || defaultRotVel == 0.0) return;
    timeNeeded =
      MAX((endTr.translation() - startTr.translation()).len()/defaultTranslVel,
	  fabs(angle)/defaultRotVel);
  }

  //make this a whole number of timesteps
  numSteps = (int)ceil(timeNeeded/myWorld->getTimeStep());
  timeNeeded = numSteps*myWorld->getTimeStep();
  DBGP("Desired Tran Traj numSteps " << numSteps);  

  if (numSteps) {
    traj.clear();
    traj.reserve(numSteps);
  }

  for (i=0;i<numSteps;i++) {
    t = (double)i/(double)(numSteps-1);
    
    coeffs[4] = 6.0 - 3.0*(startVel+endVel)*timeNeeded;
    coeffs[3] = -15.0 + (8.0*startVel + 7.0*endVel)*timeNeeded;
    coeffs[2] = 10.0 - (6.0*startVel + 4.0*endVel)*timeNeeded;
    coeffs[1] = 0.0;
    coeffs[0] = startVel*timeNeeded; 

    dist = 0.0;    
    tpow = 1.0;
    for (j=0;j<5;j++) {
      tpow *= t;
      dist += tpow*coeffs[j];
    }
    newRot = Quaternion::Slerp(dist,startTr.rotation(),endTr.rotation());
    newPos = (1.0-dist)*startTr.translation() + dist*endTr.translation();
    traj.push_back(transf(newRot,newPos));
  }
}

/*!
  This is only called when dynamics is on.  It updates the joint values
  of the dynamic joints. It is important that this be called after each 
  dynamic movement because other function then look at joint->getVal()
  for many different reasons.
*/
void
Robot::updateJointValuesFromDynamics()
{
	double *jointVals = new double[ getNumJoints() ];
	//accumulate joint values as decided by the dynamics
	for (int f=0;f<numChains;f++) {
		for (int l=0;l<chainVec[f]->getNumLinks();l++) {
			if (chainVec[f]->getLink(l)->getDynJoint())
				chainVec[f]->getLink(l)->getDynJoint()->updateValues();
		}
		for (int j=0; j<chainVec[f]->getNumJoints(); j++) {
			jointVals[ chainVec[f]->getJoint(j)->getNum() ] = chainVec[f]->getJoint(j)->getDynamicsVal();
		}
	}
	//set the joint themselves
	setJointValues(jointVals);

	//we no longer set the DOFs according to what the dynamic joints look like
	//we just do it once when the dynamics are turned off
	//however, the PD controller still needs to do it and relies on jont vals having
	//been set here
	delete [] jointVals;
}

/*!
  Applies joint friction forces as a pair of equal and opposite forces
  to links connected at each joint.  Frictional forces resist the
  direction of motion of the joint and uses a viscous friction model
  with the coefficients defined in the robot configuration file.
  
  Also applies other passive forces, such as spring forces for compliant
  joints. Spring coefficients are also read from config file.
*/
void
Robot::applyJointPassiveInternalWrenches()
{
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<getChain(c)->getNumJoints(); j++) {
			getChain(c)->getJoint(j)->applyPassiveInternalWrenches();
		}
	}
}

/*! Transparently goes to each DOF and asks them to build the limit 
	constraints however they want. See the corresponding DOF functions 
	and users manual for dynamics for details.
*/
void 
Robot::buildDOFLimitConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
								double* H, double *g, int &hcn)
{
	for (int d=0; d<numDOF; d++) {
		dofVec[d]->buildDynamicLimitConstraints(islandIndices, numBodies, H, g, hcn);
	}
}

/*! Transparently goes to each DOF and asks them to build the coupling 
	constraints however they want. See the corresponding DOF functions 
	and users manual for dynamics for details.
*/
void
Robot::buildDOFCouplingConstraints(std::map<Body*,int> &islandIndices, int numBodies, 
								   double* Nu, double *eps, int &ncn)
{	
	for(int d=0; d<numDOF; d++) {
		dofVec[d]->buildDynamicCouplingConstraints(islandIndices, numBodies, Nu, eps, ncn);
	}
	return;
}

/*!
  If the current simulation time is past the DOF setpoint update time
  this updates the DOF set points.  Then the PD position controller for
  each DOF computes the control force for the DOF given the length of the
  current timestep.
*/
void
Robot::DOFController(double timeStep)
{
	DBGP(" in Robot controller");
	if (myWorld->getWorldTime() > dofUpdateTime) {
		DBGP("Updating setpoints");
		for (int d=0;d<numDOF;d++) {
			dofVec[d]->updateSetPoint();
		}
		dofUpdateTime += myWorld->getTimeStep();
	}
	for (int d=0;d<numDOF;d++) {
		dofVec[d]->callController(timeStep);
	}
}


/*! A specialized function that attempts to see if the autograsp has completed,
	when executed dynamically. Does not do a great job at it, as this turned out
	to be a much trickier problem than expected.
*/
bool 
Robot::dynamicAutograspComplete()
{
	for (int c=0; c<numChains; c++) {
		Link *l = chainVec[c]->getLink( chainVec[c]->getNumLinks()-1 );
		//if the last link has a contact, the autograsp *might* have ended
		if (l->getNumContacts()) continue;
		int jNum = chainVec[c]->getLastJoint(chainVec[c]->getNumLinks()-1);
		Joint *j = chainVec[c]->getJoint(jNum);
		double limit;
		//which joint limit are we going towards
		if (dofVec[j->getDOFNum()]->getDefaultVelocity() > 0) {
			if (j->getCouplingRatio() > 0) {
				limit = j->getMax();
			} else {
				limit = j->getMin();
			}
		} else if (dofVec[j->getDOFNum()]->getDefaultVelocity() < 0) {
			if (j->getCouplingRatio() < 0) {
				limit = j->getMax();
			} else {
				limit = j->getMin();
			}
		} else {
			assert(0);
		}
		//if this joint has hit its limit, it is possible that the autograsp is done
		if (fabs(j->getDynamicsVal() - limit) < 3.0e-2) continue;
		//if none or the above are true, the autograsp definitely is still going
		DBGP("Autograsp going on chain " << c << " joint " << jNum << ": val " 
			 << j->getDynamicsVal() << "; limit " << limit);
		return false;
	}
	return true;		
}

/*! Looks at dynamics LCP parameters to determine if any of the contacts are slipping
	during dynamic simulation. Does not do a great job at it, as this turned out
	to be a much trickier problem than expected.
*/
bool
Robot::contactSlip()
{
	double linearThreshold = 100.0;
	double angularThreshold = 2.0;
	for (int c=0; c<numChains; c++) {
		for (int l=0; l<getChain(c)->getNumLinks(); l++) {
			Link *link = getChain(c)->getLink(l);
			if (!link->getNumContacts()) continue;
			const double *v = link->getVelocity();
			double magn = 0.0;
			for (int i=0; i<3; i++) {
				magn += v[i] * v[i];
			}
			if (magn > linearThreshold * linearThreshold) {
				DBGA("Chain " << c << " link " << l << " moving with linear vel. magnitude " << magn);
				return true;
			}
			magn = 0.0;
			for (int i=3; i<6; i++) {
				magn += v[i] * v[i];
			}
			if (magn > angularThreshold * angularThreshold) {
				DBGA("Chain " << c << " link " << l << " moving with angular vel. magnitude " << magn);
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//                           Hand Methods
//////////////////////////////////////////////////////////////////////////////

/*! Simply creates a new grasp object */
Hand::Hand(World *w,const char *name) : Robot(w,name)
{
  grasp = new Grasp(this);
}

/*! Deletes the associate grasp object */
Hand::~Hand()
{
  std::cout << "Deleting Hand: " << std::endl;
  delete grasp;
}

/*! Calls the Robot's clone fctn, but also sets this hand's grasp
	to point to the same object as the original's
*/
void
Hand::cloneFrom(Hand *original)
{
	Robot::cloneFrom(original);
	grasp->setObjectNoUpdate( original->getGrasp()->getObject() );
	grasp->setGravity( original->getGrasp()->isGravitySet() );
}

/*!
  Closes each DOF at a rate equal to \a speedFactor * its default velocity.
  The closing continues until a joint limit is reached or a contact
  prevents further motion.  The larger the \a speedFactor, the larger the
  individual steps will be and increases the likelyhood that a collision
  may be missed.  The \a renderIt flag controls whether the intermediate
  steps will be rendered.
  
  If \a stopAtContact is set and the execution is in static mode, movement
  will stop as soon as any contact is made. Otherwise, movement will continue
  untill all joints have been stopped and no movement is possible.
  
  If you want the opposite motion, just pass a negative \a speedFactor.
*/
bool
Hand::autoGrasp(bool renderIt, double speedFactor, bool stopAtContact)
{
	int i;
	double *desiredVals = new double[numDOF];

	if (myWorld->dynamicsAreOn()) {
		for (i=0;i<numDOF;i++) {
			if (speedFactor * dofVec[i]->getDefaultVelocity() > 0)
				desiredVals[i] = dofVec[i]->getMax();
			else if (speedFactor * dofVec[i]->getDefaultVelocity() < 0)
				desiredVals[i] = dofVec[i]->getMin();		
			else desiredVals[i] = dofVec[i]->getVal();
			DBGP("Desired val "<<i<<" "<<desiredVals[i]);
			//for now
			dofVec[i]->setDesiredVelocity(speedFactor * dofVec[i]->getDefaultVelocity());
		}
		setDesiredDOFVals(desiredVals);
		delete [] desiredVals;
		return true;
	}

	double *stepSize= new double[numDOF];
	for (i=0;i<numDOF;i++) {
		if (speedFactor * dofVec[i]->getDefaultVelocity() >= 0) desiredVals[i] = dofVec[i]->getMax();
		else desiredVals[i] = dofVec[i]->getMin();
		stepSize[i] = dofVec[i]->getDefaultVelocity()*speedFactor*AUTO_GRASP_TIME_STEP;
	}

	bool moved = moveDOFToContacts(desiredVals, stepSize, stopAtContact, renderIt);
	delete [] desiredVals;
	delete [] stepSize;
	return moved;
}

/*
	This is a shortcut (read "hack") that opens the hand (in the opposite direction 
	of autoGrasp), in increments of speedFactor * each DOF range. Does it until no 
	collision is detected. It does it in chunks, never does interpolation, if it can 
	not find a collision-free pose it just returns false.
*/
bool
Hand::quickOpen(double speedFactor)
{
	double *desiredVals = new double[numDOF];
	getDOFVals(desiredVals);
	double *desiredSteps = new double[numDOF];
	for (int i=0; i<numDOF; i++) {
		double d = -1 * dofVec[i]->getDefaultVelocity();
		if ( d > 0 ) {
			desiredSteps[i] = d * speedFactor * (dofVec[i]->getMax() - desiredVals[i]);
		} else if ( d < 0 ) {
			desiredSteps[i] = d * speedFactor * (desiredVals[i] - dofVec[i]->getMin() );
		} else {
			desiredSteps[i] = 0;
		}
	}
	bool done = false, success = false;
	int loops = 0;
	while (!done) {
		loops++;
		//we move out at least once
		done = true;
		for (int i=0; i<numDOF; i++) {
			desiredVals[i] += desiredSteps[i];

			if ( desiredVals[i] > dofVec[i]->getMax() ) {desiredVals[i] = dofVec[i]->getMax();}
			else if ( desiredVals[i] < dofVec[i]->getMin() ){ desiredVals[i] = dofVec[i]->getMin();}
			else if (desiredSteps[i] != 0) {done = false;} //we can do at least another loop
			//DBGA("DOF " << i << " to " << desiredVals[i]);
		}
		forceDOFVals(desiredVals);
		if ( myWorld->noCollision() ) {
			done = true;
			success = true;
		}		
	}
	if (loops > 20) DBGA("Open finger loops: " << loops << " Hand: " << myName.latin1());
	delete [] desiredVals;
	delete [] desiredSteps;
	return success;
}

/*! Moves the palm (robot) in the direction specified by approachDirection. It 
	stops if an object has been touched, or the maximum distance of \a moveDist
	has been covered. Returns true if object has been hit. It expects an initial
	state that is collision-free.
*/
bool
Hand::approachToContact(double moveDist, bool oneStep)
{
	transf newTran = translate_transf(vec3(0,0,moveDist) * getApproachTran()) * getTran();
	bool result;
	if (oneStep) {
		result = moveTo(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);
	} else {
		result = moveTo(newTran, 50*Contact::THRESHOLD, M_PI/36.0);
	}
	if (result) {
		DBGP("Approach no contact");
		return false;
	} else {
		DBGP("Approach results in contact");
		return true;
	}
}

/*! Moves the hand back along its approach direction until it is collision free, 
	then forward until it finds the exact moment of contact. It moves forward at 
	most \a moveDist. It can accept	an initial state that is in collision, as it 
	will move back until collision is resolved.
*/
bool 
Hand::findInitialContact(double moveDist)
{
	CollisionReport colReport;
	while (myWorld->getCollisionReport(&colReport)) {
		transf newTran = translate_transf(vec3(0,0,-moveDist / 2.0) * 
						 getApproachTran()) * getTran();
		setTran(newTran);
	}
	return approachToContact(moveDist, false);
}

/*! Instead of going through the robot's function for doing this, it goes to
	the Grasp class which only considers contacts against a single object (that
	is the target of the grasp).
*/
int 
Hand::accumulateQuasistaticForces(bool useDynamicDofForce)
{
	grasp->collectContacts();
	if (!grasp->getNumContacts()) return 0;
	return grasp->computeQuasistaticForces(useDynamicDofForce);
}

void 
Robot::setJointParameters(double couple, double k)
{
	for (int c=0; c<numChains; c++) {
		for (int j=0; j<chainVec[c]->getNumJoints(); j++) {
			Joint *joint = chainVec[c]->getJoint(j);
			int jn = joint->getNum();
			if (jn==1 || jn==3 || jn==5 || jn==7) {
				joint->setParameters(couple, k);
			}

		}
	}
}

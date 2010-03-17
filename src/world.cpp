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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: world.cpp,v 1.76.2.2 2009/04/27 14:33:14 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the simulation world....
 */

#include <string>
#include <algorithm>

#include <qstringlist.h>
#include <qsettings.h>
#include <qpushbutton.h>
#include <q3groupbox.h>
#include <QFile>
#include <qdatetime.h>
#include <QTextStream>

#include "myRegistry.h"
#include "matvecIO.h"
#include "world.h"
#include "worldElement.h"
#include "mytools.h"
#include "body.h"
#include "robot.h"
#include "humanHand.h"
#include "contact.h"
#include "contactSetting.h"
#include "ivmgr.h"
#include "dynamics.h"
#include "grasp.h"
#include "dynJoint.h"
#include "ivmgr.h"
#include "barrett.h"
#include "matvec3D.h"
#include "bBox.h"

#include "collisionInterface.h"

#undef PQP_COLLISION
//#undef GRASPIT_COLLISION

#ifdef PQP_COLLISION
#include "PQPCollision.h"
#endif
#ifdef BULLET_COLLISION
#include "Bullet/bulletCollision.h"
#endif
#ifdef GRASPIT_COLLISION 
#include "Graspit/graspitCollision.h"
#endif

#ifdef HARDWARE_LIB
#include "CyberGlove.h"
#include "Flock.h"
#include "FlockThread.h" //can be removed if threaded version is not used
#endif

#include <Inventor/sensors/SoIdleSensor.h>
#include <Inventor/sensors/SoTimerSensor.h>

//simulations of arches with John Ochsendorf
#include "arch.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

FILE *errFP=NULL;

//#define GRASPITDBG
#include "debug.h"

//#define PROF_ENABLED
#include "profiling.h"
PROF_DECLARE(WORLD_FIND_CONTACTS);
PROF_DECLARE(WORLD_COLLISION_REPORT);
PROF_DECLARE(WORLD_NO_COLLISION);
PROF_DECLARE(WORLD_GET_DIST);
PROF_DECLARE(WORLD_POINT_TO_BODY_DISTANCE);
PROF_DECLARE(WORLD_FIND_VIRTUAL_CONTACTS);
PROF_DECLARE(WORLD_FIND_REGION);
PROF_DECLARE(DYNAMICS);

#define MAXBODIES 256
#define TIMER_MILLISECONDS 100.0

char graspitVersionStr[] = "GraspIt! version 2.0 beta";

/*! Prepares an empty world. \a mgr is the instance of the inventor manager
	that will handle all user interaction. Also initialized collision detection
	system and reads in global settings such as friction coefficients 
*/
World::World(QObject *parent, const char *name, IVmgr *mgr) : QObject(parent,name)
{
  myIVmgr = mgr;

  numBodies = numGB = numRobots = numHands = 0;
  numSelectedBodyElements = numSelectedRobotElements = 0;
  numSelectedElements = 0;
  numSelectedBodies = 0;
  currentHand = NULL;

  isTendonSelected = false;
  selectedTendon = NULL;

  worldTime = 0.0;
  modified = false;

  allCollisionsOFF = false;
  softContactsON = true;

  cofTable=kcofTable=NULL;
  // set up the world parameters 
  readSettings();  

#ifdef PQP_COLLISION
  mCollisionInterface = new PQPCollision();
#endif
#ifdef BULLET_COLLISION
  mCollisionInterface = new BulletCollision();
#endif
#ifdef GRASPIT_COLLISION
  mCollisionInterface = new GraspitCollision();
#endif

  IVRoot = new SoSeparator;
  IVRoot->ref();

  idleSensor = NULL;
  timerSensor = NULL;
  dynamicsOn = false;

  mGlove = NULL;
  mCyberGloveOn = false;

  mFlock = NULL;
  mFlockOn = false;
  mCameraBird = 0;
}

/*! Saves the global settings and parameters and deletes the world
	along with all the objects that populate it.
*/
World::~World()
{
  int i;
  DBGP("Deleting world");

  saveSettings();

  for (i=0;i<numMaterials;i++) {
	free(cofTable[i]);
	free(kcofTable[i]);
  }
  free(cofTable);
  free(kcofTable);

  for (i=numRobots-1;i>=0;i--)
    destroyElement(robotVec[i]);

  for (i=numBodies-1;i>=0;i--) {
    DBGP("Deleting body: " << i <<" "<<bodyVec[i]->getName().latin1());
    destroyElement(bodyVec[i]);
  }
  
  if (idleSensor) delete idleSensor;
  delete mCollisionInterface;
  IVRoot->unref();

#ifdef HARDWARE_LIB
  if (mGlove) delete mGlove;
  if (mFlock) {
	  delete mFlock;
  }
#endif
}

/*! Returns the material id of a material with name \a matName 
*/
int
World::getMaterialIdx(const QString &matName) const
{
  for (int i=0; i<numMaterials; i++) {
	if (materialNames[i] == matName) return i;
  }
  return -1;
}

/*! Sets all the default (hard-coded) coefficients and parameters. If the user later
	changes them, they are saved when the world is destroyed, and automatically loaded
	later, when the application is started again.
*/
void
World::setDefaults()
{
  int i;
  dynamicsTimeStep = 0.0025;

  if (cofTable) {
	for (i=0;i<numMaterials;i++) {
	  free(cofTable[i]);
	  free(kcofTable[i]);
	}
	free(cofTable);
	free(kcofTable);
  }

  numMaterials = 7;
  cofTable = (double **)malloc(numMaterials*sizeof(double *));
  kcofTable = (double **)malloc(numMaterials*sizeof(double *));
  for (i=0;i<numMaterials;i++) {
    cofTable[i] = (double *)malloc(numMaterials*sizeof(double));
    kcofTable[i] = (double *)malloc(numMaterials*sizeof(double));
  }

  materialNames.clear();
  materialNames.push_back("frictionless");
  materialNames.push_back("glass");
  materialNames.push_back("metal");
  materialNames.push_back("plastic");
  materialNames.push_back("wood");
  materialNames.push_back("rubber");
  materialNames.push_back("stone");

  //frictionless
  for (i=0;i<7;i++) {
    cofTable[0][i] = cofTable[i][0] = 0.0;
	kcofTable[0][i] = kcofTable[i][0] = 0.0;
  }

  //stone
  cofTable[6][6] = 0.7;
  cofTable[6][1] = cofTable[1][6] = 0.3;
  cofTable[6][2] = cofTable[2][6] = 0.4;
  cofTable[6][3] = cofTable[3][6] = 0.4;
  cofTable[6][4] = cofTable[4][6] = 0.6;
  cofTable[6][5] = cofTable[5][6] = 1.5;
  kcofTable[6][6] = 0.6;
  kcofTable[6][1] = kcofTable[1][6] = 0.2;
  kcofTable[6][2] = kcofTable[2][6] = 0.3;
  kcofTable[6][3] = kcofTable[3][6] = 0.3;
  kcofTable[6][4] = kcofTable[4][6] = 0.5;
  kcofTable[6][5] = kcofTable[5][6] = 1.4;

  //glass
  cofTable[1][1] = 0.2;
  cofTable[1][2] = cofTable[2][1] = 0.2;
  cofTable[1][3] = cofTable[3][1] = 0.2;
  cofTable[1][4] = cofTable[4][1] = 0.3;
  cofTable[1][5] = cofTable[5][1] = 1.0;
  kcofTable[1][1] = 0.1;
  kcofTable[1][2] = kcofTable[2][1] = 0.1;
  kcofTable[1][3] = kcofTable[3][1] = 0.1;
  kcofTable[1][4] = kcofTable[4][1] = 0.2;
  kcofTable[1][5] = kcofTable[5][1] = 0.9;
  
  //metal
  cofTable[2][2] = 0.2;
  cofTable[2][3] = cofTable[3][2] = 0.2;
  cofTable[2][4] = cofTable[4][2] = 0.3;
  cofTable[2][5] = cofTable[5][2] = 1.0;
  kcofTable[2][2] = 0.1;
  kcofTable[2][3] = kcofTable[3][2] = 0.1;
  kcofTable[2][4] = kcofTable[4][2] = 0.2;
  kcofTable[2][5] = kcofTable[5][2] = 0.9;

  //plastic
  cofTable[3][3] = 0.3;
  cofTable[3][4] = cofTable[4][3] = 0.4;
  cofTable[3][5] = cofTable[5][3] = 1.0;
  kcofTable[3][3] = 0.2;
  kcofTable[3][4] = kcofTable[4][3] = 0.3;
  kcofTable[3][5] = kcofTable[5][3] = 0.9;

  //wood
  cofTable[4][4] = 0.4;
  cofTable[4][5] = cofTable[5][4] = 1.0;
  kcofTable[4][4] = 0.3;
  kcofTable[4][5] = kcofTable[5][4] = 0.9;

  //rubber
  cofTable[5][5] = 2.0;
  kcofTable[5][5] = 1.9;
}

/*! Reads in previously saved coefficients and parameters, if they exist. In Windows,
	they are read from the registry, in Linux they should be read from a file. It seems 
	that currently only the Window implementation is here. If they don't exist, the 
	default values are used.
*/
void
World::readSettings()
{
  QSettings settings;
  int i,j,newNumMaterials;
  double **newcofTable,**newkcofTable;
  std::vector<QString> newMaterialNames;

  setDefaults();

  settings.insertSearchPath(QSettings::Windows, COMPANY_KEY);

  dynamicsTimeStep = settings.readDoubleEntry(APP_KEY + QString("World/dynamicsTimeStep"),dynamicsTimeStep);

  newNumMaterials = settings.readNumEntry(APP_KEY + QString("World/numMaterials"),numMaterials);

  newcofTable = (double **)malloc(newNumMaterials*sizeof(double *));
  newkcofTable = (double **)malloc(newNumMaterials*sizeof(double *));
  newMaterialNames.resize(newNumMaterials, QString::null);

  for (i=0;i<newNumMaterials;i++) {
	  newcofTable[i] = (double *)malloc(newNumMaterials*sizeof(double));
	  newkcofTable[i] = (double *)malloc(newNumMaterials*sizeof(double));
	}
  
  for (i=0;i<numMaterials;i++) {
    newMaterialNames[i] = settings.readEntry(APP_KEY + QString("World/material%1").arg(i),materialNames[i]);

    for (j=i;j<numMaterials;j++) {
	  newcofTable[i][j] = newcofTable[j][i] =
	    settings.readDoubleEntry(APP_KEY + QString("World/cof%1%2").arg(i).arg(j),cofTable[i][j]);

      newkcofTable[i][j] = newkcofTable[j][i] =
	    settings.readDoubleEntry(APP_KEY + QString("World/kcof%1%2").arg(i).arg(j),kcofTable[i][j]);
	}

	for (;j<newNumMaterials;j++) {
	  newcofTable[i][j] = newcofTable[j][i] =
        settings.readDoubleEntry(APP_KEY + QString("World/cof%1%2").arg(i).arg(j));

	  newkcofTable[i][j] = newkcofTable[j][i] =
        settings.readDoubleEntry(APP_KEY + QString("World/kcof%1%2").arg(i).arg(j));
	}
  }
  for (;i<newNumMaterials;i++){
    newMaterialNames.push_back(settings.readEntry(APP_KEY + QString("World/material%1").arg(i)));

	for (j=i;j<newNumMaterials;j++) {
      newcofTable[i][j] = newcofTable[j][i] =
        settings.readDoubleEntry(APP_KEY + QString("World/cof%1%2").arg(i).arg(j));

	  newkcofTable[i][j] = newkcofTable[j][i] =
        settings.readDoubleEntry(APP_KEY + QString("World/kcof%1%2").arg(i).arg(j));
	}
  }

  for (i=0;i<numMaterials;i++) {
	free(cofTable[i]);
	free(kcofTable[i]);
  }

  free(cofTable);
  free(kcofTable);
  materialNames = newMaterialNames;
  numMaterials = newNumMaterials;
  cofTable = newcofTable;
  kcofTable = newkcofTable;
}

/*! Saves current global settings to the registry */
void
World::saveSettings()
{
  int i,j;
  QSettings settings;
  settings.insertSearchPath(QSettings::Windows, COMPANY_KEY);

  settings.writeEntry(APP_KEY + QString("World/numMaterials"),numMaterials);
  for (i=0;i<numMaterials;i++) {
	settings.writeEntry(APP_KEY + QString("World/material%1").arg(i),materialNames[i]);
	for (j=i;j<numMaterials;j++) { 
      settings.writeEntry(APP_KEY + QString("World/cof%1%2").arg(i).arg(j),cofTable[i][j]);
	  settings.writeEntry(APP_KEY + QString("World/kcof%1%2").arg(i).arg(j),kcofTable[i][j]);
	}
  }
}

/*! Removes a world element (a robot or any kind of body) from the World.
	Also removes it from the scene graph and the collision detection system.
	If \a deleteElement is true, it also deletes it at the end. If it is a 
	graspable body and there is a grasp that references it, it associates 
	the grasp with the first GB or NULL if none are left.
*/
void
World::destroyElement(WorldElement *e, bool deleteElement)
{
	std::vector<Body *>::iterator bp;
	std::vector<GraspableBody *>::iterator gp;
	std::vector<Robot *>::iterator rp;
	std::vector<Hand *>::iterator hp;
	DBGP("In remove element: " << e->className());

	if (e->inherits("Body")) {
		DBGP("found a body");
		mCollisionInterface->removeBody( (Body*) e);
		for (bp=bodyVec.begin();bp!=bodyVec.end();bp++) {
			if (*bp == e) {
				bodyVec.erase(bp); numBodies--;
				DBGP("removed body "<<((Body *)e)->getName()<<" from world");
				break;
			}
		}
		for (gp=GBVec.begin();gp!=GBVec.end();gp++)  {
			if (*gp == e) {
				GBVec.erase(gp); numGB--;
				for (hp=handVec.begin();hp!=handVec.end();hp++) {
					if ((*hp)->getGrasp()->getObject() == e) {
						if (numGB > 0)
							(*hp)->getGrasp()->setObject(GBVec[0]);
						else
							(*hp)->getGrasp()->setObject(NULL);
					}
				}
				DBGP("removed GB " << ((Body *)e)->getName()<<" from world");
				break;
			}
		}
	}

	if (e->inherits("Robot")) {
		DBGP(" found a robot");
		for (hp=handVec.begin();hp!=handVec.end();hp++)  {
			if (*hp == e) {
				handVec.erase(hp); numHands--;
				if (currentHand == e) {
					if (numHands > 0) currentHand = handVec[0];
					else currentHand = NULL;
				}
				DBGP("removed hand " << ((Robot *)e)->getName() << " from world");  
				emit handRemoved();
				break;
			}
		}
		for (rp=robotVec.begin();rp!=robotVec.end();rp++) {
			if (*rp == e) {
				robotVec.erase(rp); numRobots--;
      			DBGP("removed robot " << ((Robot *)e)->getName() << " from world"); 
				break;
			}
		}
	}

	int idx = IVRoot->findChild(e->getIVRoot());
	if (deleteElement) delete e;
	else e->getIVRoot()->ref();
	if (idx > -1)
		IVRoot->removeChild(idx);
	if (!deleteElement) e->getIVRoot()->unrefNoDelete();

	emit numElementsChanged();
	modified = true;
}

/*! Loads a simulation world file. These usually consists of one or more robots,
	graspable bodies and obstacles, each with its own transform. Optionally, 
	a camera position might also be specified. Also restores any connections
	between robots.
*/
int
World::load(const QString &filename)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    DBGA("Could not open " <<filename.latin1()<< " for reading");
    return FAILURE;
  }
  QString graspitRoot = getenv("GRASPIT");
  graspitRoot.replace("\\","/");
  if (graspitRoot.at(graspitRoot.size()-1)!='/') {
	  graspitRoot += "/";
  }

  QTextStream stream( &file );
  //version
  stream.readLine();

  QString buf, elementType, matStr, elementPath, elementName,mountFilename;
  Link *mountPiece;
  QString line;
  WorldElement *element;
  int mat;
  transf tr;
  int prevRobNum,chainNum,nextRobNum;
  bool badFile;

  while(!stream.atEnd()) {
    stream >> elementType;
    if (elementType.isEmpty()) continue;

    if (elementType == "Obstacle") {
      stream >> elementName;
      stream >> matStr;
      mat = getMaterialIdx(matStr);
      if (mat == -1) {
		QTWARNING(elementName + ": " + matStr + " is not a valid material name");
		file.close();
		return FAILURE;
      }
      elementPath = graspitRoot + elementName;
      DBGP("importing " << elementPath.latin1())
      element = importBody("Body",elementPath);
      if (!element) {
		QTWARNING("Could not open " + elementPath);
		file.close();
		return FAILURE;
      }
      if (readTransRotFromQTextStream(stream,tr)==FAILURE) {
		file.close();
		return FAILURE;
      }
      element->setTran(tr);
    }
    else if (elementType == "GraspableBody") {
      stream >> elementName;
      elementPath = graspitRoot + elementName;
      DBGP("importing " << elementPath.latin1())
      element = importBody("GraspableBody",elementPath);
      if (!element) {
	    QTWARNING("Could not open " + elementPath);
	    file.close();
	    return FAILURE;
      }
      if (readTransRotFromQTextStream(stream,tr)==FAILURE){
	  file.close();
	  return FAILURE;
      }
      element->setTran(tr);
    }
    else if (elementType == "Robot") {
      Robot *robot;
      stream >> elementPath;
      elementPath = graspitRoot + elementPath;
      DBGA("importing " << elementPath.latin1())
      if ((robot = importRobot(elementPath))==NULL){
	    QTWARNING("Could not open " + elementPath);
	    file.close();
	    return FAILURE;
      }
      element = robot;
      robot->readDOFVals(stream);
      if (readTransRotFromQTextStream(stream,tr)==FAILURE) {
	    file.close();
	    return FAILURE;
      }
      element->setTran(tr);

    }
    else if (elementType == "Connection") {
      line = stream.readLine();
      line = stream.readLine();
      QTextStream lineStream(&line,QIODevice::ReadOnly);
      lineStream >> prevRobNum >> chainNum >> nextRobNum;
	  line = stream.readLine();
      QTextStream lineStream2(&line,QIODevice::ReadOnly);
	  lineStream2 >> mountFilename;
      if (prevRobNum < 0 || prevRobNum >= numRobots || nextRobNum < 0 ||
	  nextRobNum >= numRobots || chainNum < 0 || 
	  chainNum >= robotVec[prevRobNum]->getNumChains()) {
		badFile = true; break;
	  }
      if (readTransRotFromQTextStream(stream,tr)==FAILURE) {
		badFile = true;
		QTWARNING("Error reading connection transform"); break;
      }
      DBGP("offset tran " << tr)
      if (!mountFilename.isEmpty()) {
		KinematicChain *prevChain= robotVec[prevRobNum]->getChain(chainNum);
		mountFilename = graspitRoot + mountFilename;
		DBGA("Mount filename: " << mountFilename.latin1());
	    if ((mountPiece = robotVec[nextRobNum]->importMountPiece(mountFilename))) {
		  addLink(mountPiece);
  		  toggleCollisions(false, prevChain->getLink(prevChain->getNumLinks()-1), mountPiece);
		  toggleCollisions(false, mountPiece,robotVec[nextRobNum]->getBase());
		}
		mountFilename=(char *)NULL;
	  }
      robotVec[prevRobNum]->attachRobot(robotVec[nextRobNum],chainNum,tr);
	} else if (elementType == "Camera" ) {
		double px, py, pz, q1, q2, q3, q4, fd;
		stream >> px >> py >> pz >> q1 >> q2 >> q3 >> q4;
		fd = 100;
		if (myIVmgr != NULL)
			myIVmgr->setCamera(px, py, pz, q1, q2, q3, q4,fd);
	}  else if (elementType == "CameraFull" ) {
		double px, py, pz, q1, q2, q3, q4, fd;
		stream >> px >> py >> pz >> q1 >> q2 >> q3 >> q4 >> fd;
		if (myIVmgr != NULL)
			myIVmgr->setCamera(px, py, pz, q1, q2, q3, q4, fd);
	}
    else {
      QTWARNING(elementType + " is not a valid WorldElement type");
      file.close();
      return FAILURE;
    }
  }
  file.close();
  findAllContacts();
  modified = false;
  resetDynamics();
  return SUCCESS;
}

/*! Saves this world to a file. This includes all the world elements as well as 
	positions in the world, and the positions and orientation of the camera. It
	does not save the dynamic state of the bodies.
*/
int
World::save(const QString &filename)
{
  QFile file(filename);
  int i,j,k,l;
  
 if (!file.open(QIODevice::WriteOnly)) {
    QTWARNING("could not open " + filename + "for writing");
    return FAILURE;
  }
  QTextStream stream( &file );

  stream<<"#"<<graspitVersionStr<<endl;

  for (i=0;i<numBodies;i++) {
    if (bodyVec[i]->isA("Body")) {
      stream<<"Obstacle"<<endl;
      stream<<*bodyVec[i];
      stream<< "T " << bodyVec[i]->getTran() << endl << endl;
    }
    else if (bodyVec[i]->inherits("GraspableBody")) {
      stream<<"GraspableBody"<<endl;
      stream<<*((GraspableBody *)bodyVec[i]);
      stream<< "T " << bodyVec[i]->getTran() <<endl <<endl;
    }
  }
   
  for (i=0;i<numRobots;i++) {
    stream << "Robot" << endl;
	stream << robotVec[i]->getFilename() << endl;
    robotVec[i]->writeDOFVals(stream);
	stream << endl;
    stream << "T " << robotVec[i]->getTran() <<endl <<endl;
  }

  for(i=0;i<numRobots;i++) {
    for (j=0;j<robotVec[i]->getNumChains();j++) {
      KinematicChain *chain = robotVec[i]->getChain(j);
      for (k=0;k<chain->getNumAttachedRobots();k++) {	
		stream<<"Connection"<<endl;
		stream<< i << ' ' << j <<' ';
		for (l=0;l<numRobots;l++)
			if (chain->getAttachedRobot(k) == robotVec[l]) break;
		stream << l << endl;
		if (chain->getAttachedRobot(k)->getMountPiece()) {
			stream << chain->getAttachedRobot(k)->getMountPiece()->getFilename();
		}
		stream<<endl;
		stream<< "T " << chain->getAttachedRobotOffset(k) << endl << endl;
      }
    }
  }

  stream << "CameraFull"<<endl;
  float px, py, pz, q1, q2, q3, q4, fd;
  if (myIVmgr) {
	  myIVmgr->getCamera(px, py, pz, q1, q2, q3, q4, fd);
	  stream<<px<<" "<<py<<" "<<pz<<" "<<q1<<" "<<q2<<" "<<q3<<" "<<q4<<" "<<fd<<endl<<endl;
  }

  file.close();
  modified = false;
  return SUCCESS;
}

/*! Imports a body which is loaded from a file. \bodyType must be a class name 
	from the Body hierarchy (e.g. "Body", "DynamicBody", etc). \a filename is
	the complete path to the file containing the body. The new body is created,
	loaded from the file, initialized, and added to the collision detection and 
	scene graph.
*/
Body *
World::importBody(QString bodyType,QString filename)
{ 
  Body *newBody = (Body *) WorldElement::createInstance(bodyType,this,NULL);
  if (!newBody) return NULL;
  if (newBody->load(filename)==FAILURE) return NULL;
  newBody->addToIvc();
  addBody(newBody);
  return newBody;
}

/*! Adds to this world a body that is already created, initialized and somehow
	populated. This usually means a body obtained by loading from a file or 
	cloning another body. It does NOT add the new body to the collision detection 
	system, it is the caller's responsability to do that. Also does not initialize 
	dynamics.
*/
void
World::addBody(Body *newBody)
{
  newBody->setDefaultViewingParameters();
  bodyVec.push_back(newBody);
  numBodies++;
  if (newBody->inherits("GraspableBody")) {
    GBVec.push_back((GraspableBody *)newBody);
	if (numGB == 0) {
	  for (int i=0;i<numHands;i++) {
	    handVec[i]->getGrasp()->setObject((GraspableBody *)newBody);
	  }
	}
    numGB++;
  }
  IVRoot->addChild(newBody->getIVRoot());
  modified = true;
  emit numElementsChanged();
}

/*! Adds a robot link. No need to add it to scene graph, since the robot 
	will add it to its own tree. The robot will also init dynamics and
	add the link to collision detection.
*/
void
World::addLink(Link *newLink)
{
  bodyVec.push_back(newLink);
  numBodies++;
}

/*! Loads a robot from a file and adds it to the world. \a filename must
	contain the full path to the robot.
*/
Robot *
World::importRobot(QString filename)
{
  QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
    QTWARNING("could not open " + filename);
    return NULL;
  }

  QTextStream stream( &file );
  QString line;
  while(!stream.atEnd()) {
    line = stream.readLine();
    if (!(line.isEmpty() || line[0] == '#')) break;
  }
  file.close();
  
  Robot *robot = (Robot *)WorldElement::createInstance(line,this,NULL);
  if (!robot) return NULL;
  if (robot->load(filename) == FAILURE) {
    delete robot;
    return NULL;
  }

  addRobot(robot);
  return robot;
}

/*! Adds to this world a robot that is already created, initialized and somehow
	populated. This usually means a robot obtained by loading from a file or 
	cloning another body. It only adds the new robot to the scene graph of
	\a addToScene is true. Can be used when robots are clones and clones are set
	to work, but we don't necessarily want to see them.
	
	Collisions between links in the same chain are always disables, as geometry
	files are usually imperfect and as joints move, the meshes then to hit each
	other at the joints, which would make the robot unusable.
*/
void
World::addRobot(Robot *robot, bool addToScene)
{
	robotVec.push_back(robot);
	numRobots++;

	if (robot->getBase()) {
		addLink(robot->getBase());
	}

	for (int f=0; f<robot->getNumChains(); f++) {
		for (int l=0; l<robot->getChain(f)->getNumLinks(); l++) {
			addLink(robot->getChain(f)->getLink(l));
		}
	}
	for (int f=0; f<robot->getNumChains(); f++) {
		mCollisionInterface->activatePair(robot->getChain(f)->getLink(0), robot->getBase(), false);   
		for (int l=0; l<robot->getChain(f)->getNumLinks(); l++) {
			for(int l2 = 0; l2<robot->getChain(f)->getNumLinks();l2++) {
				if (l==l2) continue;
				mCollisionInterface->activatePair(robot->getChain(f)->getLink(l), 
												  robot->getChain(f)->getLink(l2), false);
			}
		}
	}
	if (robot->inherits("Hand")) {    
		handVec.push_back((Hand *)robot);
		if (numGB > 0) ((Hand *)robot)->getGrasp()->setObject(GBVec[0]);
		numHands++;
		if (numHands==1) setCurrentHand((Hand *)robot);
	}
	for (int d=0;d<robot->getNumDOF();d++) {
		robot->getDOF(d)->setDesiredPos(robot->getDOF(d)->getVal());
	}

	if (addToScene) {
		addElementToSceneGraph(robot);
	}

	modified = true;
	emit numElementsChanged();
}


/*! Removes a robot from the world and also deletes it. */
void
World::removeRobot(Robot *robot)
{
	std::vector<Robot *>::iterator rp;
	std::vector<Hand *>::iterator hp;
	for (rp=robotVec.begin();rp!=robotVec.end();rp++) {
		if (*rp == robot) {
			robotVec.erase(rp);
			numRobots--;
			break;
		}
	}
	if (robot->inherits("Hand")) {
		for (hp=handVec.begin();hp!=handVec.end();hp++) {
			if (*hp == robot) {
				handVec.erase(hp);
				numHands--;
				break;
			}
		}
	}
	delete robot;
}

/*! Adds an element to the Scene Graph; the element must already be part of the
	world. Used when removeElementFromSceneGraph has previously been called on
	this element.
*/
void
World::addElementToSceneGraph(WorldElement *e)
{
	if (IVRoot->findChild( e->getIVRoot() ) >= 0) {
		DBGA("Element is already in scene graph");
		return;
	}
	int i;
	if (e->inherits("Robot")) {
		for (i=0; i<(int)robotVec.size(); i++) {
			if (robotVec[i] == e) break;
		}
		if ( i==robotVec.size() ) {
			DBGA("Robot not a part of the world");
			return;
		}
	} else if (e->inherits("Body")) {
		for (i=0; i<(int)bodyVec.size(); i++) {
			if (GBVec[i] == e) break;
		}	
		if ( i==GBVec.size() ) {
			DBGA("Body not a part of the world");
			return;
		}
	}
	IVRoot->addChild( e->getIVRoot() );
}

/*! Remove an element that is part of this world from the Scene Graph; 
	the element remains a part of the world and can be used in simulations; 
	it is just not rendered anymore.
*/
void
World::removeElementFromSceneGraph(WorldElement *e)
{
	int c = IVRoot->findChild( e->getIVRoot() );
	if (c<0) {
		DBGA("Element not part of the scene graph");
		return;
	}
	e->getIVRoot()->ref();
	IVRoot->removeChild(c);
	e->getIVRoot()->unrefNoDelete();
}

/*! Makes a static element into a dynamic body by creating a DynamicBody
	from it and initializing default dynamic properties. The geometry is 
	not loaded again, but it is added to the collision detection again.
*/
DynamicBody *
World::makeBodyDynamic(Body *b, double mass)
{ 
  DynamicBody *dynBod = new DynamicBody(*b,mass);
  dynBod->addToIvc();
  addBody(dynBod);
  destroyElement(b, true);
  findContacts(dynBod);
  return dynBod;
}

/*! Selects a world element. If the element is a Robot, also selects all of its
	links, plus the base. If it is a Body, it is also added to the list of
	selectedBodies.
*/
void
World::selectElement(WorldElement *e)
{
  std::list<WorldElement *>::iterator ep;
  int c,l;

  DBGP("selecting element "<<e->getName().latin1());
  if (e->inherits("Body")) {DBGP(" with collision id " << ((Body*)e)->getId());}

  if (e->inherits("Body")) numSelectedBodyElements++;
  else if (e->inherits("Robot")) numSelectedRobotElements++;
  numSelectedElements++;

  selectedElementList.push_back(e);

  selectedBodyVec.clear();
  for (ep=selectedElementList.begin();ep!=selectedElementList.end();ep++) {
    if ((*ep)->inherits("Body")) selectedBodyVec.push_back((Body *)(*ep));
    else if ((*ep)->inherits("Robot")) {
      Robot *r = (Robot *)(*ep);
      selectedBodyVec.push_back(r->getBase());
	  for (c=0;c<r->getNumChains();c++){
	    for (l=0;l<r->getChain(c)->getNumLinks();l++) {
	      selectedBodyVec.push_back(r->getChain(c)->getLink(l));
		}
	  }
    }
  }
  numSelectedBodies = selectedBodyVec.size();

  DBGP("selected elements "<<numSelectedElements);
  DBGP("selected bodies "<<numSelectedBodies);

  emit selectionsChanged();
}

/*! Deselects a world element. If the element is a Robot, also deselects all of 
	its links, plus the base.
*/
void
World::deselectElement(WorldElement *e)
{
  std::list<WorldElement *>::iterator ep;
  int c,l;

  DBGP("deselecting element "<<e->getName().latin1());
  if (e->inherits("Body")) numSelectedBodyElements--;
  else if (e->inherits("Robot")) numSelectedRobotElements--;
  numSelectedElements--;

  selectedElementList.remove(e);

  selectedBodyVec.clear();
  for (ep=selectedElementList.begin();ep!=selectedElementList.end();ep++) {
    if ((*ep)->inherits("Body")) selectedBodyVec.push_back((Body *)(*ep));
    else if ((*ep)->inherits("Robot")) {
      Robot *r = (Robot *)(*ep);
      selectedBodyVec.push_back(r->getBase());
      for (c=0;c<r->getNumChains();c++)
	for (l=0;l<r->getChain(c)->getNumLinks();l++)
	  selectedBodyVec.push_back(r->getChain(c)->getLink(l));
    }
  }
  numSelectedBodies = selectedBodyVec.size();
  DBGP("selected elements "<<numSelectedElements);
  DBGP("selected bodies "<<numSelectedBodies);
  emit selectionsChanged();
}

/*! Clears the list of selected element and deselects all */
void
World::deselectAll()
{
  selectedElementList.clear();
  selectedBodyVec.clear();

  numSelectedElements = numSelectedBodyElements = numSelectedRobotElements = 0;
  numSelectedBodies = 0;
  emit selectionsChanged();
}

/*! Checks whether an element is currently selected by looking in the 
	selectedElementList
*/
bool
World::isSelected(WorldElement *e) const
{
  std::list<WorldElement *>::const_iterator ep;
  for (ep=selectedElementList.begin();ep!=selectedElementList.end();ep++)
    if (*ep == e) return true;
  return false;
}

/*! Toggles all collisions. If \a on is false, all collisions in the world are
	disabled. If \a on is true, they are re-enabled.
*/
void
World::toggleAllCollisions(bool on)
{
	DBGA("TOGGLING COLLISIONS");
	bool off = !on;
	if (numSelectedElements == 0) {
		allCollisionsOFF = off;
	} else if (numSelectedElements == 2) {
		if (off) toggleCollisions(false, selectedElementList.front(), selectedElementList.back());
		else toggleCollisions(true, selectedElementList.front(),  selectedElementList.back());
	} else {
		std::list<WorldElement *>::iterator ep;
		for (ep=selectedElementList.begin();ep!=selectedElementList.end();ep++) {
			if (off) toggleCollisions(false, *ep);
			else toggleCollisions(true, *ep);
		}
	}
	findAllContacts();
}

bool
World::robotCollisionsAreOff(Robot *r1, WorldElement *e)
{
	if (e->inherits("Body")) {
		Body *b1 = (Body*) e;
		if (mCollisionInterface->isActive( b1, r1->getBase() ) ) {
			return false;
		}
		for (int f=0; f<r1->getNumChains(); f++) {
			for (int l=0; l<r1->getChain(f)->getNumLinks(); l++) {
				if ( mCollisionInterface->isActive(b1, r1->getChain(f)->getLink(l)) )
					return false;
			}
		}
		return true;
	} else if (e->inherits("Robot")) {
		Robot *r2 = (Robot*) e;
		if ( mCollisionInterface->isActive(r1->getBase(), r2->getBase()) )
			return false;

		for (int f2=0; f2<r2->getNumChains(); f2++) 
		  for (int l2=0; l2<r2->getChain(f2)->getNumLinks(); l2++)
			if ( mCollisionInterface->isActive(r1->getBase(), r2->getChain(f2)->getLink(l2) ) )
			  return false;

		for (int f=0; f<r1->getNumChains(); f++) 
		  for (int l=0; l<r1->getChain(f)->getNumLinks(); l++) { 
			if ( mCollisionInterface->isActive(r1->getChain(f)->getLink(l),r2->getBase() ) )
			  return false;
		    for (int f2=0; f2<r2->getNumChains(); f2++) 
		      for (int l2=0; l2<r2->getChain(f2)->getNumLinks(); l2++)
				if ( mCollisionInterface->isActive(r1->getChain(f)->getLink(l), r2->getChain(f2)->getLink(l2) ) )
				  return false;
		  }
		return true;
	}
	return true;
}

/*! Checks whether collisions are off. Options for passing arguments are the same
	as for toggleCollisions.
*/
bool
World::collisionsAreOff(WorldElement *e1,WorldElement *e2)
{
  int f,l;
  Body *b1,*b2;
  Robot *r1,*r2;

  if (e1 == NULL) {
	  assert(e2 == NULL);
	  return allCollisionsOFF;
  }
  if (e1->inherits("Body")) {
	b1 = (Body *)e1;
	if (e2) {
	  if (e2->inherits("Body")) {
		b2 = (Body *)e2;  
		return !mCollisionInterface->isActive(b1, b2);
	  } else if (e2->inherits("Robot")) {
	    r2 = (Robot*)e2;
	    return robotCollisionsAreOff(r2,e1);
	  } else {
		  return false;
	  }
	} else return !mCollisionInterface->isActive(b1);
  } else if (e1->inherits("Robot")) {
	r1 = (Robot *)e1;
	if (e2) {
		return robotCollisionsAreOff(r1,e2);
	} else {
	  if ( mCollisionInterface->isActive(r1->getBase()) ) return false;
  	  for (f=0;f<r1->getNumChains();f++) 
		for (l=0;l<r1->getChain(f)->getNumLinks();l++)
		  if ( mCollisionInterface->isActive(r1->getChain(f)->getLink(l)) )
			return false;
    }
  }
  return true;	
}

/*! If \a e2 is NULL, is toggles collisions for \a e1 in general. Otherwise, it
	toggles collisions between \a e1 and \a e2. Remember that toggling collisions
	for a robot means applying the change to all of its links and the base
*/
void
World::toggleCollisions(bool on, WorldElement *e1,WorldElement *e2)
{
  int f,l,f2,l2;
  Body *b1,*b2;
  Robot *r1,*r2;

  assert(e1);
  if (e1->inherits("Body")) {
    b1 = (Body *)e1;
    if (e2) {
      if (e2->inherits("Body")) {
			b2 = (Body *)e2;
			mCollisionInterface->activatePair(b1,b2,on);
      } else if (e2->inherits("Robot")) {
		r2 = (Robot *)e2;
		mCollisionInterface->activatePair(b1,r2->getBase(),on);
		for (f=0;f<r2->getNumChains();f++) 
			for (l=0;l<r2->getChain(f)->getNumLinks();l++)
				mCollisionInterface->activatePair(b1, r2->getChain(f)->getLink(l), on);
	  }	    
	}
	else mCollisionInterface->activateBody(b1,on);
  } else if (e1->inherits("Robot")) {
	r1 = (Robot *)e1;
    if (e2) {
      if (e2->inherits("Body")) {
		b2 = (Body *)e2;
		mCollisionInterface->activatePair(r1->getBase(), b2, on);
		for (f=0;f<r1->getNumChains();f++) 
			for (l=0;l<r1->getChain(f)->getNumLinks();l++)
				mCollisionInterface->activatePair(r1->getChain(f)->getLink(l), b2, on);
      }
      else if (e2->inherits("Robot")) {
		r2 = (Robot *)e2;
		mCollisionInterface->activatePair(r1->getBase(), r2->getBase(), on);

		for (f2=0;f2<r2->getNumChains();f2++) 
			for (l2=0;l2<r2->getChain(f2)->getNumLinks();l2++)
				mCollisionInterface->activatePair(r1->getBase(), r2->getChain(f2)->getLink(l2),on);

		for (f=0;f<r1->getNumChains();f++) 
			for (l=0;l<r1->getChain(f)->getNumLinks();l++) {
				mCollisionInterface->activatePair(r1->getChain(f)->getLink(l), r2->getBase(), on);

				for (f2=0;f2<r2->getNumChains();f2++) 
					for (l2=0;l2<r2->getChain(f2)->getNumLinks();l2++)
						mCollisionInterface->activatePair(r1->getChain(f)->getLink(l), r2->getChain(f2)->getLink(l2), on);
			}
	  }
	}else {
		mCollisionInterface->activateBody(r1->getBase(),on);
		for (f=0;f<r1->getNumChains();f++) 
			for (l=0;l<r1->getChain(f)->getNumLinks();l++)
				mCollisionInterface->activateBody(r1->getChain(f)->getLink(l),on);	
	}
  }
}

/*! Returns true if the element \a e is not colliding with anything else. 
	Attempts to do it fast by returning as soon as any collision is found, 
	and only looking at potential collisions that involve \a e.
*/
bool
World::noCollision(WorldElement *e)
{
	PROF_TIMER_FUNC(WORLD_NO_COLLISION);
	if (allCollisionsOFF) return true;

	if (!e) {
		if (mCollisionInterface->allCollisions(CollisionInterface::FAST_COLLISION,NULL,NULL)) return false;
		return true;
	}

	std::vector<Body*> interestList;
	if (e->inherits("Body")) {
		interestList.push_back( (Body*)e );
	} else if (e->inherits("Robot")) {
		Robot *r = (Robot*)e;
		for (int c=0; c < r->getNumChains(); c++) {
			for (int l=0; l < r->getChain(c)->getNumLinks(); l++) {
				interestList.push_back( r->getChain(c)->getLink(l) );
			}
		}
		interestList.push_back( r->getBase() );
		if (r->getMountPiece()) interestList.push_back(r->getMountPiece());
	} else {
		DBGA("Unknown case in World::noCollision");
	}

	int col = mCollisionInterface->allCollisions(CollisionInterface::FAST_COLLISION, NULL, &interestList);
	if (col) return false;
	return true;
}

/*! Returns a full collision report for the bodies in the \a interestList.
	Check collision interface documentation for details on what the report
	contains. If \a interestList is NULL, is returns a collision report
	for all the bodies in the world.
*/
int
World::getCollisionReport(CollisionReport *colReport, const std::vector<Body*> *interestList)
{
  PROF_TIMER_FUNC(WORLD_COLLISION_REPORT);
  DBGP("Get COLLISION REPORT")
  colReport->clear();
  if (allCollisionsOFF) return 0;
  
  int numCols;
  numCols = mCollisionInterface->allCollisions(CollisionInterface::ALL_COLLISIONS, colReport, interestList);
  return numCols;
}

void 
World::getBvs(Body *b, int depth, std::vector<BoundingBox> *bvs)
{
	mCollisionInterface->getBoundingVolumes(b, depth, bvs);
}

/*! Returns a vector that is the shortest distance from the point \a p 
	to the body \a b. If \a normal is not NULL, it is set to the surface
	normal of body \a b at the point closest to \a p. Everything is 
	expressed in world coordinates.
*/
vec3
World::pointDistanceToBody(position p, Body *b, vec3 *normal)
{
	PROF_TIMER_FUNC(WORLD_POINT_TO_BODY_DISTANCE);
	position cp; vec3 cn;
	mCollisionInterface->pointToBodyDistance(b, p, cp, cn);
	if (normal) {
		*normal = cn;
	}
	return cp - p;
}

/*! Returns the distance between two bodies \a b1 and \a b2 */
double
World::getDist(Body *b1,Body *b2)
{
	PROF_TIMER_FUNC(WORLD_GET_DIST);
	position p1,p2;
	return mCollisionInterface->bodyToBodyDistance(b1,b2,p1,p2);
}

/*! Returns the distance between two bodies \a b1 and \a b2 and sets
	\a p1 and \a p2 to the locations of the two points, one on each
	body, that are closest to each other
*/
double
World::getDist(Body *b1,Body *b2, position &p1, position &p2)
{
	PROF_TIMER_FUNC(WORLD_GET_DIST);
	return mCollisionInterface->bodyToBodyDistance(b1,b2,p1,p2);
}

void
World::findVirtualContacts(Hand *hand, Body *object)
{
	PROF_TIMER_FUNC(WORLD_FIND_VIRTUAL_CONTACTS);
	ContactReport contactSet;
	for (int f=0; f<hand->getNumFingers(); f++) {
		for (int l=0; l<hand->getFinger(f)->getNumLinks(); l++) {
			Link *link = hand->getFinger(f)->getLink(l);
			link->breakVirtualContacts();
			ContactData pc = findVirtualContact(link, object);
			contactSet.clear();
			contactSet.insert( contactSet.begin(), pc);
			addVirtualContacts(link, f, l, object, contactSet, false);
		}
	}

	hand->getPalm()->breakVirtualContacts();
	ContactData pc = findVirtualContact(hand->getPalm(), object);
	contactSet.clear();
	contactSet.insert( contactSet.begin(), pc);
	addVirtualContacts(hand->getPalm(), -1, 0, object, contactSet, false);
}

ContactData
World::findVirtualContact(Link *link, Body *object)
{
	position p1, p2;
	getDist(link, object, p1, p2);
	vec3 n = p1 * link->getTran() - p2 * object->getTran();
	n = normalise(n);
	n = n * link->getTran().inverse();

	return ContactData(p1, p2, n, -n) ;
}

/*! Asks each grasp to update itself (i.e. recompute its wrench spaces,
	presumably due to some change in  contact geometry.
*/
void
World::updateGrasps()
{
	bool graspChanged = false;
	for (int i=0;i<numHands;i++) {
		if (handVec[i]->contactsChanged()) {
			handVec[i]->getGrasp()->update();
			graspChanged = true;
		}
	}
		if (graspChanged) {
		emit graspsUpdated();
	}
}

/*! Finds all the contacts between the bodies listed in the \a colReport. 
	Usually, the \a colReport is populated by the caller, based on a set
	of bodies that were in collision before, but now are assumed to be out
	of collision and potentially just touching.
	
	A contact occurs when bodies are separated by a distance less than the 
	contact threshold.  This routine uses the collision detection system to 
	find the points of contact between each body and at each one creates a 
	pair of contact objects which are added to the individual bodies.
*/
void
World::findContacts(CollisionReport &colReport)
{
	PROF_TIMER_FUNC(WORLD_FIND_CONTACTS);
	CollisionReport::iterator it = colReport.begin();
	while(it!=colReport.end()) {
		CollisionReport::iterator it2;
		bool duplicate = false;
		for (it2 = colReport.begin(); it2<it; it2++) {
			if ( (*it).first == (*it2).first && (*it).second == (*it2).second) {
				duplicate = true;
				break;
			}
		}
		if (duplicate) {
			DBGP("duplicate: " << (*it).first->getName().latin1() << "--" <<  (*it).second->getName().latin1());
			it = colReport.erase(it);
			continue;
		}

		if ( getDist( (*it).first, (*it).second ) > Contact::THRESHOLD ) {
			DBGP("no contact: " << (*it).first->getName().latin1() << "--" << (*it).second->getName().latin1());
			it = colReport.erase(it);
			continue;
		}
		it++;
	}

	ContactReport cres;
	for (int i=0; i<(int)colReport.size(); i++) {
		std::list<Contact*> contacts = colReport[i].first->getContacts();
		std::list<Contact*>::iterator it;
		for (it = contacts.begin(); it!=contacts.end(); it++) {
			if ( (*it)->getBody1() == colReport[i].first && (*it)->getBody2() == colReport[i].second) {
				colReport[i].first->removeContact(*it);
			}
		}
		cres.clear();
		mCollisionInterface->contact(&cres, Contact::THRESHOLD, colReport[i].first, colReport[i].second);
		addContacts(colReport[i].first, colReport[i].second, cres, softContactsAreOn());
	}
}

/*! Finds all the contact on body \a b. A pair of contact objects is created 
	for each contact and are added to the individual bodies that are in contact.
*/
void
World::findContacts(Body *b)
{
	PROF_TIMER_FUNC(WORLD_FIND_CONTACTS);
	ContactReport contactReport;
	for (int i=0;i<numBodies;i++) {
		if (bodyVec[i] != b) {
			mCollisionInterface->contact(&contactReport, Contact::THRESHOLD, b, bodyVec[i]);
			addContacts(b, bodyVec[i], contactReport, softContactsAreOn());
		}
	}
}

/*! Finds and adds all the contacts between any two bodies in the world. 
	A pair of contact objects is created for each contact and are added 
	to the individual bodies that are in contact.
*/
void
World::findAllContacts()
{
	PROF_TIMER_FUNC(WORLD_FIND_CONTACTS);
	for (int i=0;i<numBodies;i++) {
		bodyVec[i]->resetContactList();
	}
  	if (allCollisionsOFF) return;
	CollisionReport report;
	int numContacts;
	numContacts = mCollisionInterface->allContacts(&report, Contact::THRESHOLD, NULL);
	DBGP("found " << numContacts << " contacts. Adding...");
 	for (int i=0;i<numContacts;i++) {
		addContacts( report[i].first, report[i].second, report[i].contacts, softContactsAreOn());
		DBGP( report[i].first->getName().latin1() << " - " << report[i].second->getName().latin1() );
	 }
}


/*! Takes a point and sends it to the collision detection system to find 
	the neighborhood of that point on a body, used in soft contacts for the
	input for the paraboloid fitter.
*/
void World::FindRegion( const Body *body, position point, vec3 normal, double radius,
						Neighborhood *neighborhood)
{
	PROF_TIMER_FUNC(WORLD_FIND_REGION);
	mCollisionInterface->bodyRegion(body, point, normal, radius, neighborhood);
}

/*! Starts dynamic simulation. This is all the user has to do; from now
	on, dynamic steps are triggered automatically by an idle sensor.
	The only way to determine the motion of the bodies is to set desired
	values for the robot dofs and let the world time step computations and
	the robot dof controllers do the rest.
	
	Use the resetDynamics routine to fix the base robot of every collection 
	of robots so that it does not fall under gravity and to set the desired 
	pose of each robot to be it's current state so that the controllers try
    to maintain the current positions until the user requests something
	different.
*/
void
World::turnOnDynamics()
{
	//PROF_RESET_ALL;
	//PROF_START_TIMER(DYNAMICS);
	dynamicsOn = true;
	if (idleSensor) delete idleSensor;
	idleSensor = new SoIdleSensor(dynamicsCB,this);
	idleSensor->schedule();
}

/*! Pauses dynamic simulation; no more time steps are computed*/
void
World::turnOffDynamics()
{
	//PROF_STOP_TIMER(DYNAMICS);
	//PROF_PRINT_ALL;
	if (idleSensor) delete idleSensor;
	idleSensor = NULL;
	dynamicsOn = false;
	for (int i=0; i<numRobots; i++) {
		//actually set joint values
		robotVec[i]->updateJointValuesFromDynamics();
		//try to approximate robot dof values based on where the joints ended up
		robotVec[i]->updateDOFFromJoints(NULL);
	}
	updateGrasps();
}

/*! Resets the velocities and accelerations of all bodies; fixes the base 
	robot of every collection of robots so that it is not affected by gravity 
	and sets the desired pose of each robot to be it's current state so that 
	the controllers try to maintain the current positions until the user 
	requests something different.
*/
void World::resetDynamics()
{
	int i,d;
	std::vector<double> zeroes(6,0.0);
	//clear the dynamic stack of all objects
	for (i=0;i<numBodies;i++) {
		if (bodyVec[i]->isDynamic()) {
			((DynamicBody *)bodyVec[i])->clearState();
			((DynamicBody *)bodyVec[i])->setAccel(&zeroes[0]);
			((DynamicBody *)bodyVec[i])->setVelocity(&zeroes[0]);
		}
	}

	//push the initial dynamic state
	pushDynamicState();

	for (i=0;i<numRobots;i++) {
		//fix robot bases
		robotVec[i]->getBaseRobot()->getBase()->fix();
		//set robot desired position to current position
		for (d=0;d<robotVec[i]->getNumDOF();d++) {
			robotVec[i]->getDOF(d)->setDesiredPos(robotVec[i]->getDOF(d)->getVal());
		}
	}   
}

void
World::dynamicsCB(void *data,SoSensor *)
{
  World *myWorld = (World *)data;
  myWorld->stepDynamics();
}

/*! Saves the current dynamic state (velocities and accelerations of
	all bodies) onto a stack.
*/
void
World::pushDynamicState()
{
  for (int i=0;i<numBodies;i++)
    if (bodyVec[i]->isDynamic())
      ((DynamicBody *)bodyVec[i])->pushState();
}

/*! Restores the dynamic state currently at the top of the stack */
void
World::popDynamicState()
{
	bool stackEmpty = false;
	for (int i=0;i<numBodies;i++) {
		if (bodyVec[i]->isDynamic()) {
			if (!((DynamicBody *)bodyVec[i])->popState()) stackEmpty = true;
		}
	}
	if (stackEmpty) {
		DBGA("Resetting dynamics");
		resetDynamics();
	}
}

/*! One of the two main functions of the dynamics time step. This function is 
	called to move the bodies according to the velocities and accelerations found
	in the previous step. The move is attempted for the duration of the time step 
	given in \a timeStep.
	
	After all the bodies are moved, then collisions are checked. If any collisions
	are found, the move is traced back and the value of the times step is 
	interpolated until the exact moment of contact is found. The actual value
	of the time step until contact is made is returned. If interpolation fails,
	a negative actual time step is returned. All the resulting contacts are added
	to the bodies that are in contact to be used for subsequent computations.
	
	The same procedure is carried out if, by executing a full time step, a joint
	ends up outside of its legal range.
*/
double
World::moveDynamicBodies(double timeStep)
{
	int i,numDynBodies,numCols,moveErrCode;
	std::vector<DynamicBody *> dynBodies;
	static CollisionReport colReport;
	bool jointLimitHit;
	double contactTime,delta,tmpDist,minDist,dofLimitDist;

	//save the initial position
	for (i=0;i<numBodies;i++) {
		if (bodyVec[i]->isDynamic()) {
			dynBodies.push_back((DynamicBody *)bodyVec[i]);
			((DynamicBody *)bodyVec[i])->markState();
		}
	}
	numDynBodies = dynBodies.size();
 
	//call to the dynamics engine to perform the move by the full time step
    DBGP("moving bodies with timestep: " << timeStep);
	moveErrCode = moveBodies(numDynBodies,dynBodies,timeStep);
	if (moveErrCode == 1){ // object out of bounds
		popDynamicState();
		turnOffDynamics();
		return -1.0 ;
	}

	//this sets the joints internal values according to how bodies have moved
	for (i=0;i<numRobots;i++) {
		robotVec[i]->updateJointValuesFromDynamics();
	}

	//check if we have collisions
	if (numDynBodies > 0) numCols = getCollisionReport(&colReport);
	else numCols = 0;
	  
	//check if we have joint limits exceeded
	jointLimitHit = false;
	for (i=0;i<numRobots;i++) {
		if (robotVec[i]->jointLimitDist() < 0.0) jointLimitHit = true;
	}

	//if so, we must interpolate until the exact moment of contact or limit hit
	if (numCols || jointLimitHit) {
		//return to initial position
		for (i=0;i<numDynBodies;i++) {
			dynBodies[i]->returnToMarkedState();
		}
		minDist = 1.0e+255;
		dofLimitDist = 1.0e+255;

#ifdef GRASPITDBG
		if (numCols) {
			std::cout << "COLLIDE!" << std::endl;
			for (i=0;i<numCols;i++) {
				std::cout << colReport[i].first->getName() << " collided with " << 
					colReport[i].second->getName() << std::endl;
			}

			for (i=0;i<numCols;i++) {
				tmpDist = getDist(colReport[i].first,colReport[i].second);
				if (tmpDist < minDist) minDist = tmpDist;	
				std::cout << "minDist: " << tmpDist <<" between " << std::endl;
				std::cout << colReport[i].first->getName() << " and " <<
				colReport[i].second->getName() << std::endl;
			}      
		}
#endif      

		//this section refines the timestep until the objects are separated
		//by a distance less than CONTACT_THRESHOLD
		bool done = false;
		contactTime = timeStep;
		delta = contactTime/2;
		contactTime -= delta;
  
		while (!done) {
			delta /= 2.0;	  
			for (i=0;i<numDynBodies;i++) {
				dynBodies[i]->returnToMarkedState();
			}
		    DBGP("moving bodies with timestep: " << contactTime);
			moveErrCode = moveBodies(numDynBodies,dynBodies,contactTime);
      
			if (moveErrCode == 1){ // object out of bounds
				popDynamicState();
				turnOffDynamics();
				return -1.0;
			}

			const char *min_body_1,*min_body_2;

			//this computes joints values according to how dynamic bodies have moved
			for (i=0;i<numRobots;i++) {
				robotVec[i]->updateJointValuesFromDynamics();
			}

			if (numCols) {
				minDist = 1.0e+255;
				for (i=0;i<numCols;i++) {
					tmpDist = getDist(colReport[i].first,colReport[i].second);
					if (tmpDist < minDist) {
						minDist = tmpDist;
						min_body_1 = colReport[i].first->getName().latin1();
						min_body_2 = colReport[i].second->getName().latin1();
						DBGP("minDist: " << minDist << " between " << colReport[i].first->getName() << 
							" and " << colReport[i].second->getName());
					}
				}
			}

			if (jointLimitHit) {
				dofLimitDist = 1.0e10;
				for (i=0; i<numRobots; i++) {
					dofLimitDist = MIN( dofLimitDist, robotVec[i]->jointLimitDist() );
				}
			}

			if (minDist <= 0.0 || dofLimitDist < -resabs)
				contactTime -= delta;
			else if (minDist > Contact::THRESHOLD * 0.5 && dofLimitDist > 0.01) //why is this not resabs
 				contactTime += delta;
			else break;
	       
			if (fabs(delta) < 1.0E-15 || contactTime < 1.0e-7) {
				if (minDist <= 0) {
					fprintf(stderr,"Delta failsafe due to collision: %s and %s\n",min_body_1,min_body_2);
				} else {
					fprintf(stderr,"Delta failsafe due to joint\n");
				}
				done = true;  // failsafe
			}
		}

		// COULD NOT FIND COLLISION TIME
		if (done && contactTime < 1.0E-7) {
			DBGP("!! could not find contact time !!");
			for (i=0;i<numDynBodies;i++)
				dynBodies[i]->returnToMarkedState();    
		}
		worldTime += contactTime;
	}
	else { // if no collision
		worldTime += timeStep;
		contactTime = timeStep;
	}
       
#ifdef GRASPITDBG
	std::cout << "CHECKING COLLISIONS AT MIDDLE OF STEP: ";
	numCols = getCollisionReport(colReport);

	if (!numCols){ 
		std::cout << "None." << std::endl;
	}
	else {
		std::cout << numCols <<" found!!!" << std::endl;
		for (i=0;i<numCols;i++) {
			std::cout << colReport[i].first->getName() << " collided with " <<
			colReport[i].second->getName() << std::endl;
		}
	}
#endif

	if (numDynBodies > 0)
		findAllContacts();

	for (i=0; i<numRobots; i++){
		if ( robotVec[i]->isA("HumanHand") )
			((HumanHand*)robotVec[i])->updateTendonGeometry();
	}
	emit tendonDetailsChanged();

	if (contactTime<1.0E-7) return -1.0;
	return contactTime;
}

/*! Asks the dynamics engine to compute the velocities of all bodies at
	the current time step. These will be used in the next time step when
	the bodies are moved by World::moveDynamicBodies.
	
	The bodies are separated into "islands" of bodies connected by contacts 
	or joints.	Two dynamic bodies are connected if they share a contact or
    a joint.  Then for each island, this calls the iterate dynamics routine
    to build and solve the LCP to find the velocities of all the bodies
    in the next iteration.
*/	
int
World::computeNewVelocities(double timeStep)
{
  bool allDynamicsComputed;
  static std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  std::vector<DynamicBody *> robotLinks;
  std::vector<DynamicBody *> dynIsland;
  std::vector<Robot *> islandRobots;
  int i,j,numLinks,numDynBodies,numIslandRobots,lemkeErrCode;

#ifdef GRASPITDBG
  int islandCount = 0;
#endif

  do {
    // seed the island with one dynamic body
    for (i=0;i<numBodies;i++)
      if (bodyVec[i]->isDynamic() &&
		!((DynamicBody *)bodyVec[i])->dynamicsComputed()) {

        // if this body is a link, add all robots connected to the link
        if (bodyVec[i]->inherits("Link")) {
	      Robot *robot = ((Robot *)((Link *)bodyVec[i])->getOwner())->getBaseRobot();
	      robot->getAllLinks(dynIsland);
	      robot->getAllAttachedRobots(islandRobots);
		}
        else
	      dynIsland.push_back((DynamicBody *)bodyVec[i]);
        break;
	  }
    numDynBodies = dynIsland.size();
    for (i=0;i<numDynBodies;i++)
      dynIsland[i]->setDynamicsFlag();

  // add any bodies that contact any body already in the dynamic island
    for (i=0;i<numDynBodies;i++) {
      contactList = dynIsland[i]->getContacts();
      for (cp=contactList.begin();cp!=contactList.end();cp++) {
	//if the contacting body is dynamic and not already in the list, add it
	    if ((*cp)->getBody2()->isDynamic() &&
	      !((DynamicBody *)(*cp)->getBody2())->dynamicsComputed()) {
	      DynamicBody *contactedBody = (DynamicBody *)(*cp)->getBody2();
	  
	      // is this body is a link, add all robots connected to the link
	      if (contactedBody->isA("Link")) {
	        Robot *robot = ((Robot *)((Link *)contactedBody)->getOwner())->getBaseRobot();
	        robot->getAllLinks(robotLinks);
	        robot->getAllAttachedRobots(islandRobots);
	        numLinks = robotLinks.size();
	        for (j=0;j<numLinks;j++)
	          if (!robotLinks[j]->dynamicsComputed()) {
		        dynIsland.push_back(robotLinks[j]);
		        robotLinks[j]->setDynamicsFlag();
		        numDynBodies++;
			  }
	        robotLinks.clear();
		  }
	      else {
	        dynIsland.push_back(contactedBody);
	        contactedBody->setDynamicsFlag();
	        numDynBodies++;
		  }
		}
	  }
    }

    numIslandRobots = islandRobots.size();

#ifdef GRASPITDBG
    std::cout << "Island "<< ++islandCount<<" Bodies: ";
    for (i=0;i<numDynBodies;i++)
      std::cout << dynIsland[i]->getName() <<" ";
    std::cout << std::endl;
    std::cout << "Island Robots"<< islandCount<<" Robots: ";
    for (i=0;i<numIslandRobots;i++)
      std::cout << islandRobots[i]->getName() <<" ";
    std::cout << std::endl << std::endl;
#endif  
    
    for (i=0;i<numDynBodies;i++)
      dynIsland[i]->markState();

	DynamicParameters dp;
	if (numDynBodies > 0) {
		dp.timeStep = timeStep;
		dp.useContactEps = true;
		dp.gravityMultiplier = 1.0;
		lemkeErrCode = iterateDynamics(islandRobots,dynIsland,&dp);

		if (lemkeErrCode == 1){ // dynamics could not be solved
	        std::cerr << "LCP COULD NOT BE SOLVED!"<<std::endl<<std::endl;
			turnOffDynamics();
			return -1;
		}
	}

    dynIsland.clear(); 
    islandRobots.clear();
    allDynamicsComputed = true;
    for (i=0;i<numBodies;i++)
      if (bodyVec[i]->isDynamic() &&
	  !((DynamicBody *)bodyVec[i])->dynamicsComputed()) {
	allDynamicsComputed = false;
	break;
      }
  }  while (!allDynamicsComputed);

  // clear all the dynamicsComputed flags
  for (i=0;i<numBodies;i++)
    if (bodyVec[i]->isDynamic())
      ((DynamicBody *)bodyVec[i])->resetDynamicsFlag();

  /*  double conMaxErr = 0.0;

  if (!errFP) errFP=fopen("constraintError.txt","w");
  fprintf(errFP,"%le ",worldTime);
  for (i=0;i<numBodies;i++) {
    contactList = bodyVec[i]->getContacts();
    for (cp=contactList.begin();cp!=contactList.end();cp++) {
	  conMaxErr = MAX(conMaxErr,fabs((*cp)->getConstraintError()));
	}
	
  }
  fprintf(errFP,"%le ",conMaxErr);
  fprintf(errFP," ");
  int k,l;
  double jointErrMax=0.0;
  for (i=0;i<numRobots;i++) {
	if (robotVec[i]->getBase()->getDynJoint())
      for (j=0;j<3;j++)
		jointErrMax = MAX(jointErrMax,fabs(robotVec[i]->getBase()->getDynJoint()->getConstraintError()[j]));
        //fprintf(errFP,"%le ",robotVec[i]->getBase()->getDynJoint()->getConstraintError()[j]);
	for (j=0;j<robotVec[i]->getNumChains();j++) {
      KinematicChain *chain=robotVec[i]->getChain(j);
	  for (k=0;k<chain->getNumLinks();k++)
        if (chain->getLink(k)->getDynJoint())
	  	  for (l=0;l<3;l++)
			jointErrMax = MAX(jointErrMax,fabs(robotVec[i]->getBase()->getDynJoint()->getConstraintError()[j]));
           // fprintf(errFP,"%le ",chain->getLink(k)->getDynJoint()->getConstraintError()[l]);
	}
  }
  fprintf(errFP,"%le",jointErrMax);
  fprintf(errFP,"\n");

  */

  emit dynamicStepTaken();
  return 0;
}

void
World::resetDynamicWrenches()
{
	for (int i=0; i<numBodies; i++) {
		if (bodyVec[i]->isDynamic()) {
			((DynamicBody*)bodyVec[i])->resetExtWrenchAcc();
		}
	}
}

/*! A complete dynamics step:
	- moveDynamicBodies moves the bodies according to velocities computed
	at the previous time step.
	
	- active and passive joint forces are applied; contacts are detected
	
	- computeNewVelocities computes the velocities according to contact and
	joint constraints, for the next time step.
*/
void
World::stepDynamics()
{
	resetDynamicWrenches();
	double actualTimeStep = moveDynamicBodies(dynamicsTimeStep);
	if (actualTimeStep<0) {
		turnOffDynamics();
		emit dynamicsError("Timestep failsafe reached.");
		return;
	}

	for (int i=0; i<numRobots; i++) {
		robotVec[i]->DOFController(actualTimeStep);
		robotVec[i]->applyJointPassiveInternalWrenches();
	}

	if (computeNewVelocities(actualTimeStep)) {
		emit dynamicsError("LCP could not be solved.");
		return;
	} 
    if (idleSensor) idleSensor->schedule();
}

void World::selectTendon(Tendon *t)
{
	if (isTendonSelected)
		selectedTendon->deselect();

	isTendonSelected = true;
	selectedTendon = t;
	selectedTendon->select();

	//we need to change selected hand to the hand that owns currently selected tendon
	//so we can populate the drop-down tendon list in the GUI
	if ( getCurrentHand() != (Hand*)selectedTendon->getRobot() )
	{
		setCurrentHand( (Hand*)t->getRobot() );
		emit handSelectionChanged();
	}

	emit tendonSelectionChanged();
}

void World::selectTendon(int i)
{
	if (isTendonSelected)
		selectedTendon->deselect();

	// i is supposed to be an index into the currently selected hand's list of tendons
	if (!currentHand)
	{
		printf("ERROR: no hand selected\n");
		return;
	}
	if (! currentHand->isA("HumanHand") )
	{
		printf("ERROR: selected hand is not tendon-actuated\n");
		return;
	}

	if ( ((HumanHand*)currentHand)->getNumTendons() <= i)
	{
		printf("ERROR: selected hand has fewer tendons than passed parameter\n");
		return;
	}

	if (isTendonSelected)
		selectedTendon->deselect();

	isTendonSelected = true;
	selectedTendon = ((HumanHand*)currentHand)->getTendon(i);
	selectedTendon->select();
	emit tendonSelectionChanged();
}

void World::deselectTendon()
{
	isTendonSelected = false;
	if (selectedTendon)
		selectedTendon->deselect();
	selectedTendon = NULL;
	emit tendonSelectionChanged();
}

int World::getCurrentHandNumberTendons()
{
	if (!currentHand)
		return 0;
	if (! currentHand->isA("HumanHand") )
		return 0;

	return ((HumanHand*)currentHand)->getNumTendons();
}

QString World::getSelectedHandTendonName(int i)
{
	/*return the name of the i-th tendon of the currently selected hand*/
	if (i>=getCurrentHandNumberTendons())
		return QString("Error reading name");

	return ((HumanHand*)currentHand)->getTendon(i)->getName();
}

/*! Turns on the CyberGlove. An idle sensor then wakes up periodically to
	check the glove status and calls the glove callback, where all the 
	processing is actually done.
*/
void World::turnOnCyberGlove() 
{
#ifndef HARDWARE_LIB
	//sensor use is disabled
	mCyberGloveOn = false;
	return;
#else
	if (!mGlove) {
		mGlove = new CyberGlove();
	}
	if (!mGlove->testGlove()) {
		QTWARNING("Cyberglove error");
		delete mGlove; mGlove = NULL;
		mCyberGloveOn = false;
		return;
	}
	for ( int i=0; i<getNumRobots(); i++) {
		if (robotVec[i]->useCyberGlove())
			robotVec[i]->setGlove(mGlove);
	}
	mCyberGloveOn = true;

	//schedule calls to timer callback that checks cyberglove
	if (!timerSensor) {
		timerSensor = new SoTimerSensor(timerCB, this);
	}
	if (!timerSensor->isScheduled()) {
		timerSensor->setInterval( SbTime( 0.001 * TIMER_MILLISECONDS) );
		timerSensor->schedule();
	}
#endif
}

/*! The CyberGlove is no longer queried */
void World::turnOffCyberGlove()
{
	mCyberGloveOn = false;
}

/*! Turns on the Flock of Birds. An idle sensor is set to wake up periodically to
	check the Flock status and call the Flock callback, where all the processing
	is actually done.
*/
void World::turnOnFlock()
{
	// todo: dialog for all the flock and cyberglove settings
#ifndef HARDWARE_LIB
	mFlockOn = false;
	return;
#else
	//comment this out if the camera is not to be run by a bird...
	//mCameraBird = 2;

	if (!mFlock) {
		mFlock = new Flock(1);
//		mFlock = new FlockThread(1);
//		mFlock->startThread();
	}
	mFlockOn = true;
	setFlockBase(1);

	//schedule calls to timer callback that checks flock
	if (!timerSensor) {
		timerSensor = new SoTimerSensor(timerCB, this);
	}
	if (!timerSensor->isScheduled()) {
		timerSensor->setInterval( SbTime( 0.001 * TIMER_MILLISECONDS) );
		timerSensor->schedule();
	}
#endif
}

void World::turnOffFlock()
{
	mFlockOn = false;
}

/*! Based in which sensors are on, calls the Flock or CyberGlove callback
	which process their respective sensor inputs.
*/
void World::timerCB(void *data, SoSensor*)
{
	World *myWorld = (World *)data;
	bool needed = false;
	if (myWorld->cyberGloveOn()) {
		myWorld->processCyberGlove();
		needed = true;
	}
	if (myWorld->flockOn()) {
		myWorld->processFlock();
		needed = true;
	}
	if (!needed) {
		myWorld->timerSensor->unschedule();
	}
}

/*! CyberGlove callback. Instructs all robots that are connected to the
	CyberGlove to get a new reading and react accordingly.
*/
void World::processCyberGlove()
{
#ifdef HARDWARE_LIB
	if (!mGlove->instantRead()) {
		QTWARNING("Error reading CyberGlove\n");
		turnOffCyberGlove();
		return;
	}

	for ( int i=0; i<getNumRobots(); i++) {
		if (robotVec[i]->useCyberGlove()) {
				robotVec[i]->processCyberGlove();
		}		
	}
#endif
}

/*! Helper function that reads in a raw bird transform and returns it
	as a GraspIt transf.
*/
transf World::getBirdTran(int b)
{
#ifdef HARDWARE_LIB
	if (!mFlock->instantRead(b) ) {
		QTWARNING("Error reading Flock!");
		turnOffFlock();
		return transf::IDENTITY;
	}
	
	double r[9], t[3];
	mFlock->getRotationMatrix(r);
	mFlock->getPosition(t);

	transf birdTran;
	birdTran.set(mat3(r), vec3(t));
	return birdTran;
#else
	(void*)&b;
	return transf::IDENTITY;
#endif
}

/*! Flock of birds callback. Sets the transforms of all bodies connected to birds
	according to the transforms read from the bird. Also sets the transform of the 
	camera, if it is connected to a bird. Also see setFlockBase.
*/
void World::processFlock()
{
#ifdef HARDWARE_LIB
	bool absoluteBird = false;
	// If the Flock is attached to a real Barrett, we ignore readings
	// while the motors of the Barrett are active, as they corrupt the bird.
	if ( getCurrentHand() && getCurrentHand()->isA("Barrett") ) {
		if ( ((Barrett*)getCurrentHand())->isBusy() ) return;
	}
	transf birdTran,bodyTran;

	if (mCameraBird != 0) {
		birdTran = getBirdTran( mCameraBird );
		myIVmgr->setCameraTransf( mCameraFlockTran.get(birdTran) );
	}

	for (int i=0; i<getNumBodies(); i++) {
		//process only free bodies here, not robot links
		if (getBody(i)->usesFlock() && getBody(i)->getOwner() == getBody(i) ) {
			birdTran = getBirdTran( getBody(i)->getBirdNumber() );
			if (absoluteBird) {	
				bodyTran = getBody(i)->getFlockTran()->getAbsolute(birdTran);
			} else {
				bodyTran = getBody(i)->getFlockTran()->get(birdTran);
			}
			getBody(i)->moveTo(bodyTran , WorldElement::ONE_STEP, WorldElement::ONE_STEP );
		}
	}
	
	for (int i=0; i<getNumRobots(); i++) {
		if ( getRobot(i)->usesFlock() ) {
			birdTran = getBirdTran( getRobot(i)->getBirdNumber() ); 
			if (absoluteBird) {
				bodyTran = getRobot(i)->getFlockTran()->getAbsolute( birdTran );
			} else if (mCameraBird != 0) {
				bodyTran = getRobot(i)->getFlockTran()->get2( birdTran );
			} else {
				bodyTran = getRobot(i)->getFlockTran()->get( birdTran );
			}
			getRobot(i)->moveTo( bodyTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP );
		}
	}
#endif
}

/*!	The bodies that are NOT run by the master bird need to be re-arranged
	relative to the one that IS run by it. This keeps consistency between 
	the real world and the graspit world: if two objects are spaced at 
	distance d in the real world, their graspit counterparts will also be 
	spaced at distance d. So one of the bodies run by the master bird is the 
	master body, that does not change position in the graspit world when 
	we turn on the flock.
*/
void World::setFlockBase(int masterBird)
{
	(void*)&masterBird;
#ifdef HARDWARE_LIB
	if (!mFlock || !mFlockOn) {
		return;
	}

	if (mCameraBird != 0) {
		if (masterBird != mCameraBird)
			fprintf(stderr,"Master bird override: the camera bird (%d) is the master bird\n",mCameraBird);
		masterBird = mCameraBird;
	}

	transf flockTran = getBirdTran(masterBird);
	transf objectBaseTran;

	if (mCameraBird!=0){
		//in this case, everything gets rearranged relative to camera position,
		//which is set as the master bird by default
		mat3 R( vec3(0,1,0), vec3(0,0,-1), vec3(-1,0,0) );
		transf mount;
		mount.set(R, vec3(0,0,0));
		mCameraFlockTran.setMount(mount.inverse());

		objectBaseTran = myIVmgr->getCameraTransf();
		mCameraFlockTran.setFlockBase(flockTran);
		mCameraFlockTran.setObjectBase(objectBaseTran);

		objectBaseTran = mount.inverse() * objectBaseTran;
		
	} else {
		//here, we pick a random object run by the master bird and rearrange everybody rel. to it
		for (int i=0; i<getNumBodies(); i++) {
			if ( getBody(i)->getBirdNumber() == masterBird) {
				objectBaseTran = getBody(i)->getTran();
				break;
			}
		}
	}

	for (int i=0; i<getNumBodies(); i++) {
		//only bodies processed here, not robot links
		if (getBody(i)->usesFlock() && getBody(i)->getOwner() == getBody(i) ) {
			getBody(i)->getFlockTran()->setFlockBase( flockTran );
			//getBody(i)->getFlockTran()->setObjectBase( getBody(i)->getTran() );
			getBody(i)->getFlockTran()->setObjectBase( objectBaseTran );
		}
	}

	for (int i=0; i<getNumRobots(); i++) {
		if ( getRobot(i)->usesFlock() ) {
			getRobot(i)->getFlockTran()->setFlockBase( flockTran );
			if (mCameraBird == 0) {
				//if there's no camera bird, each robot is it's own master
				getRobot(i)->getFlockTran()->setObjectBase( getRobot(i)->getTran() );
			} else {
				getRobot(i)->getFlockTran()->setObjectBase( objectBaseTran );
			}			
		}
	}
#endif
}

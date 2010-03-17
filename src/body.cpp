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
// $Id: body.cpp,v 1.52.2.1 2009/04/24 22:19:19 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the body class hierarchy.
*/

#include <iostream>
#include <iomanip>
#include <QFile>
#include <QString>
#include <QTextStream>

#include "body.h"
#include "collisionInterface.h"
#include "bBox.h"
#include "triangle.h"
#include "world.h"
#include "robot.h"
#include "joint.h"
#include "dynJoint.h"
#include "ivmgr.h"
#include "contact.h"
#include "graspitGUI.h"
extern "C" {
#include "maxdet.h"
}
#include "mytools.h"

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/draggers/SoRotateDiscDragger.h>
#include <Inventor/nodekits/SoWrapperKit.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define AXES_SCALE 100.0

const float Body::CONE_HEIGHT = 20.0;
double DynamicBody::defaultMass = 300.0;

/*!
  Constructs an empty body with its material set to invalid, and the
  geometry pointer set to NULL.  Use \c load() to initialize the body from
  a model file.
*/
Body::Body(World *w,const char *name) : WorldElement(w,name)
{
	//defaults:
	mIsElastic = false;
	youngMod = -1;
	
	material = -1;
	numContacts=0;
	showFC = false;
	showVC = false;
	IVGeomRoot = NULL; IVTran = NULL; IVMat = NULL; IVContactIndicators=NULL;
	IVBVRoot = NULL;
#ifdef GEOMETRY_LIB
	IVPrimitiveRoot = NULL;
#endif
	mUsesFlock = false;
	mBirdNumber = 0;
	mRenderGeometry = true;

	initializeIV();
}

/*!
  Copy constructor remains protected (should not be called by user)
*/
Body::Body(const Body &b) : WorldElement(b)
{
  mIsElastic = b.mIsElastic;
  material = b.material;
  youngMod = b.youngMod;
  showFC = b.showFC;
  mUsesFlock = b.mUsesFlock;
  mRenderGeometry = true;
  Tran = b.Tran;

  IVRoot = b.IVRoot;
  IVTran = b.IVTran;
  IVContactIndicators = b.IVContactIndicators;
#ifdef GEOMETRY_LIB
  IVPrimitiveRoot = b.IVPrimitiveRoot;
#endif
  IVBVRoot = b.IVBVRoot;
  IVGeomRoot = b.IVGeomRoot;
  IVMat = b.IVMat;

  createAxesGeometry();
}

/*! Breaks all contacts; it is up to the world to also remove it from 
	the collision detection system or the scene graph. In general, do 
	not delete bodies directly, but use World::removeElement(...)instead 
	which can also delete them.
 */
Body::~Body()
{
  breakContacts();
  DBGP("Deleting Body: " << myName.latin1());
}

/*! Clones this body from an original. This means that the two bodies have independent
	transform, materials, properties, etc, BUT they share the scene graph geometry and
	the collision detection geometry. They can still be used for collision detection 
	independently as they can have different transforms.
	
	WARNING: cloning mechanism is incomplete. If the original is deleted, the clone 
	becomes unpredicateble and can cause the system to crash
*/
void 
Body::cloneFrom(const Body* original)
{
	mIsElastic = original->mIsElastic;
	youngMod = original->youngMod;
	material = original->material;

	//add virtual contacts
	virtualContactList.clear();
	std::list<Contact*> vc = original->getVirtualContacts();
	std::list<Contact*>::iterator it;
	for (it = vc.begin(); it!=vc.end(); it++) {
		VirtualContact *newContact = new VirtualContact( (VirtualContact*)(*it) );
		newContact->setBody(this);
		addVirtualContact(newContact);
	}

	setRenderGeometry( original->getRenderGeometry() );

	int numGeomChildren = original->getIVGeomRoot()->getNumChildren();
	//create a CLONE of all geometry
	for (int i=0; i<numGeomChildren; i++) {
		IVGeomRoot->addChild( original->getIVGeomRoot()->getChild(i) );
	}
	addIVMat(true);


	//add a CLONE to collision detection
	cloneToIvc(original);
	setTran(original->getTran());
}


/*!
  Opens and loads a body geometry file.  \a filename is the complete path to
  the body file, which should be in Inventor format or VRML.  The body is also 
  added to the collision detection system. 
*/
int 
Body::load(const QString &filename)
{
	QFile file(filename);
	QString line,defStr,valueStr;
	SoInput myInput;

	myFilename = relativePath(filename, getenv("GRASPIT"));
	setName(filename.section('/',-1).section('.',0,0));

	if (!file.open(QIODevice::ReadOnly)) {
		QTWARNING("Could not open " + filename);
		return FAILURE;
	}
	QTextStream stream( &file );

	while ( !stream.atEnd() ) 
	{
		line = stream.readLine();
		line.stripWhiteSpace();
		//only process inventor comment lines
		if (line[0]!='#')
			continue;
		defStr = line.section('#',1).section(' ',0,0);

		//material line
		if (defStr=="material")	{
			valueStr = line.section(' ',1,1);
			if (!valueStr.isEmpty()) {
				material = myWorld->getMaterialIdx(valueStr);
				if (material==-1) {
					QTWARNING("invalid material type in body file: " + myFilename);
					file.close();
					return FAILURE;
				}
			}
		}

		//Young's modulus line
		if (defStr=="youngs"){
			valueStr = line.section(' ',1,1);
			youngMod = valueStr.toDouble();
			if (youngMod <= 0) {
				QTWARNING("invalid Young's modulus in body file: " + myFilename);
				file.close();
				return FAILURE;
			}
			mIsElastic = true;
		}

		if (defStr=="useFlockOfBirds") {
			valueStr = line.section(' ',1,1);
			mBirdNumber = valueStr.toDouble();
			mUsesFlock = true;
			DBGA("Object using Flock of Birds sensor " << mBirdNumber);
		}
	}
	file.close();
 
	if (material==-1) {
		DBGA("No material type found; using default.");
		material = myWorld->getMaterialIdx("wood");
	}

	if (!myInput.openFile(filename.latin1())) {
		QTWARNING("Could not open Inventor file " + filename);
		return FAILURE;
	}

	//we will read the geomety from the file
	IVRoot->removeChild(IVGeomRoot);
	//read geometry from file
	if (myInput.isFileVRML2()) {
		IVGeomRoot = SoDB::readAllVRML(&myInput);
	} else {
		IVGeomRoot = SoDB::readAll(&myInput);
	}
	myInput.closeFile();
	if (IVGeomRoot == NULL) {
		QTWARNING("A problem occurred while reading Inventor file" + filename);
		return FAILURE;
	}
	//and add it to scene graph
	IVRoot->addChild(IVGeomRoot);
	//add material for controlling transparency
	addIVMat();
	return SUCCESS;
}

/*! After the geometry has been set, this function adds a new Material 
	node after any Material node already present so we can change the 
	transparency of this object. Doesn't seem to always work...
*/
void 
Body::addIVMat(bool clone)
{
	IVMat = new SoMaterial;
	IVMat->diffuseColor.setIgnored(true);
	IVMat->ambientColor.setIgnored(true);
	IVMat->specularColor.setIgnored(true);
	IVMat->emissiveColor.setIgnored(true);
	IVMat->shininess.setIgnored(true);

	if (clone) {
		//clone's IVMat really does nothing except die with the clone
		IVGeomRoot->addChild(IVMat);		
	} else {
		SoSearchAction *sa = new SoSearchAction;
		sa->setType(SoMaterial::getClassTypeId());
		sa->setInterest(SoSearchAction::ALL);
		sa->apply(IVGeomRoot);
		
		if (sa->getPaths().getLength() == 0)
			IVGeomRoot->insertChild(IVMat,0);
		else for (int i=0;i<sa->getPaths().getLength();i++) {
				SoGroup *g = (SoGroup *)sa->getPaths()[i]->getNodeFromTail(1);
		if (((SoMaterial *)sa->getPaths()[i]->getTail())->transparency[0] == 0.0f)
			g->insertChild(IVMat,sa->getPaths()[i]->getIndexFromTail(0)+1);	
			}
		delete sa;
	}
}

/*! Adds the body to the world's collision detection system
*/
void 
Body::addToIvc()
{
	myWorld->getCollisionInterface()->addBody(this);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
}

/*!	Clones the original's body geometry for the world
	collision detection system. The new body only gets 
	its own IVC transform.
*/
void 
Body::cloneToIvc(const Body *original)
{
	myWorld->getCollisionInterface()->cloneBody(this, original);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
}

void 
Body::setDefaultViewingParameters()
{
	showFC = false;
	showVC = false;
	setTransparency(0.0);	
}

/*! Initialized the empty scene graph structure that we will use
	in the future to render this body
*/
void 
Body::initializeIV()
{
	IVRoot = new SoSeparator;
    IVTran = new SoTransform;
    IVRoot->insertChild(IVTran,0);

	IVContactIndicators = new SoSeparator;
    IVRoot->addChild(IVContactIndicators);

#ifdef GEOMETRY_LIB
	IVPrimitiveRoot = new SoSeparator;
	IVRoot->addChild(IVPrimitiveRoot);
#endif

	IVBVRoot = new SoSeparator;
	IVRoot->addChild(IVBVRoot);

	IVGeomRoot = new SoGroup;
	IVRoot->addChild(IVGeomRoot);

	createAxesGeometry();
}

/*! Shows a bounding box hierarchy for this body. Used for 
	debug purposes.
*/
void
Body::setBVGeometry(const std::vector<BoundingBox> &bvs)
{
	IVBVRoot->removeAllChildren();
	int mark = 0;
	for (int i=0; i<(int)bvs.size(); i++) {
		SoSeparator *bvSep = new SoSeparator;

		SoMaterial *bvMat = new SoMaterial;
		bvSep->addChild(bvMat);
		float r,g,b;
		//random colors
		r = ((float)rand()) / RAND_MAX;
		g = ((float)rand()) / RAND_MAX;
		b = ((float)rand()) / RAND_MAX;

		//mark collisions	
		if (bvs[i].mMark) {
			mark++;
			r = 0.8f; g=0.0f; b=0.0f;
		} else {
			r = g = b = 0.5f;
		}
		
		bvMat->diffuseColor = SbColor(r,g,b);
		bvMat->ambientColor = SbColor(r,g,b);
		bvMat->transparency = 0.5;

		SoTransform* bvTran = new SoTransform;
		bvs[i].getTran().toSoTransform(bvTran);
		bvSep->addChild(bvTran);

		
		//a single cube for the entire box
		SoCube *bvBox = new SoCube;
		bvBox->width = 2 * bvs[i].halfSize.x();
		bvBox->height = 2 * bvs[i].halfSize.y();
		bvBox->depth = 2 * bvs[i].halfSize.z();
		bvSep->addChild(bvBox);
		

		//2 cubes so we also see separarion plane
		/*
		SoCube *bvBox = new SoCube;
		bvBox->width =		bvs[i].halfSize.x();
		bvBox->height = 2 * bvs[i].halfSize.y();
		bvBox->depth = 2 * bvs[i].halfSize.z();
		SoTransform *halfCubeTran = new SoTransform;
		halfCubeTran->translation.setValue(bvs[i].halfSize.x()/2.0, 0.0, 0.0);
		bvSep->addChild(halfCubeTran);
		bvSep->addChild(bvBox);
		halfCubeTran = new SoTransform;
		halfCubeTran->translation.setValue(-bvs[i].halfSize.x(), 0.0, 0.0);
		bvSep->addChild(halfCubeTran);
		bvSep->addChild(bvBox);
		*/

		IVBVRoot->addChild(bvSep);
	}
	DBGA("Setting bv geom: " << bvs.size() << " boxes. Marked: " << mark);
}

/*!
  Returns the current transparency value for the body (between 0 and 1).
  \sa setTransparency()
*/
float
Body::getTransparency() const
{
  return IVMat->transparency[0];
}  
  

/*!
  Set the current transparency value for the body.
  \a t is a value between 0 and 1, where 0 is opaque, 1 is transparent.
  \sa getTransparency()
*/
void
Body::setTransparency(float t)
{
  IVMat->transparency = t;
}

  
/*!
  Set the current material of the body to mat
  \sa getMaterial()
*/
void
Body::setMaterial(int mat)
{
  std::list<Contact *>::iterator cp;

  material = mat;
  if (showFC || showVC) IVContactIndicators->removeAllChildren();

  for (cp=contactList.begin();cp!=contactList.end();cp++) {
    (*cp)->updateCof();
    (*cp)->getMate()->updateCof();
    (*cp)->getBody2()->redrawFrictionCones();
  }
  redrawFrictionCones();
}


/*!
  Sets whether friction cones should be shown for this body.
*/
void
Body::showFrictionCones(bool on, int vc)
{
  showFC = on;
  if (vc==1) showVC = true;
  else if (vc==2) showVC = false;
  redrawFrictionCones();
}


/*!
  Recomputes all the friction cones on the body
*/
void
Body::redrawFrictionCones()
{
  std::list<Contact *>::iterator cp;

  IVContactIndicators->removeAllChildren();
  if (showFC) {
    for (cp=contactList.begin();cp!=contactList.end();cp++)
		IVContactIndicators->addChild( (*cp)->getVisualIndicator() );
  }
  if (showVC) {
	for (cp = virtualContactList.begin(); cp!=virtualContactList.end(); cp++)
		IVContactIndicators->addChild( ( (VirtualContact*)(*cp) )->getVisualIndicator() );
  }
}

/*! Sets whether a change of this body's transform should automatically
	trigger a redraw. This seems to not always work...
*/
void
Body::setRenderGeometry(bool s)
{
	assert(IVTran);
	mRenderGeometry = s;
	if (!s) {
//		IVRoot->enableNotify(false);
//		IVTran->enableNotify(false);
//		IVMat->enableNotify(false);
//		IVGeomRoot->enableNotify(false);
//		IVContactIndicators->enableNotify(false);
		IVTran->translation.enableNotify(false);
		IVTran->rotation.enableNotify(false);
	} else {
//		IVRoot->enableNotify(true);
//		IVTran->enableNotify(true);
//		IVMat->enableNotify(true);
//		IVGeomRoot->enableNotify(true);
//		IVContactIndicators->enableNotify(true);
		IVTran->translation.enableNotify(true);
		IVTran->rotation.enableNotify(true);
	}
	
}

/*!
  Sets the current world pose of the body to \a tr.  Collisions are not
  checked.
*/
int
Body::setTran(transf const &tr)
{
	if (tr == Tran) return SUCCESS;
	breakContacts();

	if (!myWorld->wasModified() && tr != Tran) {
		myWorld->setModified();
	}
	Tran = tr;
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
	if (IVTran) {
		Tran.toSoTransform(IVTran);
	}
	return SUCCESS;
}

 
/*!
  Given a motion relative to body coordinates, this determines whether
  the current contacts allow that motion.
*/
bool
Body::contactsPreventMotion(const transf& motion) const
{
  std::list<Contact *>::iterator cp;
  std::list<Contact *> contactList;

  contactList = getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++) {
    if ((*cp)->preventsMotion(motion)) {
      return true;
    }
  }
  return false;
}

/*!
  Breaks all contacts on the body, deleting entire contact list, and removes
  all friction cones if necessary. Also clears the list of contacts from the
  previous time step.
*/
void
Body::breakContacts()
{
	std::list<Contact *>::iterator cp;

	if (!contactList.empty()) {
		setContactsChanged();
		for (cp=contactList.begin();cp!=contactList.end();cp++) {
			delete *cp; *cp = NULL;
		}
		contactList.clear();
	}
	numContacts = 0;

	for (cp = prevContactList.begin(); cp!=prevContactList.end(); cp++) {
		if ( (*cp)->getMate() != NULL ) {
			(*cp)->getMate()->setMate(NULL);
		}
		(*cp)->setMate(NULL);
		delete *cp; *cp = NULL;
	}
	prevContactList.clear();

	if (showFC)
		IVContactIndicators->removeAllChildren();
}

/*! Removes all virtual contacts. For now, this kills all contact indicators,
	including those from non-virtual contacts...
*/
void
Body::breakVirtualContacts()
{
	std::list<Contact *>::iterator cp;
	for (cp = virtualContactList.begin(); cp!= virtualContactList.end(); cp++) {
		delete *cp; *cp = NULL;
	}
	virtualContactList.clear();
	//this should only delete virtual contact visual indicators....
	if (showVC) {
		IVContactIndicators->removeAllChildren();
	}
}

/*!
  Moves all contacts to the prevContactList and clears the contactList for new contacts
  Old contacts from prevContactList are deleted.
*/
void
Body::resetContactList()
{
	std::list<Contact *>::iterator cp;
	for (cp = prevContactList.begin(); cp!=prevContactList.end(); cp++) {
		if ( (*cp)->getMate() != NULL ) {
			(*cp)->getMate()->setMate(NULL);
		}
		(*cp)->setMate(NULL);
		delete *cp; *cp = NULL;
	}
	prevContactList.clear();
	if (!contactList.empty()) {
		setContactsChanged();
		for (cp=contactList.begin();cp!=contactList.end();cp++) {
			prevContactList.push_back(*cp);
		}
		contactList.clear();
	}
	numContacts = 0;
	if (showFC)
		IVContactIndicators->removeAllChildren();
}

/*! 
  Adds contact \a c to the body's contact list.  Assumes the contact is not
  already in the list.
  \sa removeContact()
*/
void
Body::addContact(Contact *c)
{
	setContactsChanged();
	contactList.push_back(c);
	numContacts++;
	if (showFC) {
		assert(IVContactIndicators->getNumChildren() > numContacts-2);
		IVContactIndicators->insertChild( c->getVisualIndicator(), numContacts-1 );
	}
}

/*! The number of contacts on this body. If \a b is not null, it only counts
	contacts against \a b. If it is null, is returns all contacts on this body,
	regardless of who they are against.
*/
int 
Body::getNumContacts(Body *b) const
{
	if (!b) return numContacts;
	int c = 0;
	std::list<Contact*>::const_iterator it;
	for (it = contactList.begin(); it!=contactList.end(); it++) {
		if ( (*it)->getBody2() == b) c++;
	}
	return c;
}

/*! Adds a virtual contacts to this body */
void
Body::addVirtualContact(Contact *c)
{
	setContactsChanged();
	virtualContactList.push_back(c);
	if (showVC)
		IVContactIndicators->addChild( c->getVisualIndicator() );
}

/*!
  Checks if this contact inherits some other contact from the prevContactList.
  The check looks of contact points are close, normals agree and the second 
  body is the same. If inheritance is found, some properites of the contact 
  are passed from the previous contact to this one.
*/
Contact*
Body::checkContactInheritance(Contact *c)
{
	std::list<Contact *>::iterator cp;
	bool inheritance = false;
	for (cp = prevContactList.begin(); cp != prevContactList.end(); cp++) {
		if ( (*cp)->getBody1() != c->getBody1() )
			continue;
		if ( (*cp)->getBody2() != c->getBody2() )
			continue;
		vec3 d = (*cp)->getPosition() - c->getPosition();
		if (d.len() > Contact::INHERITANCE_THRESHOLD )
			continue;
		vec3 n1 = (*cp)->getNormal();
		vec3 n2 = c->getNormal();
		double theta = n1 % n2;
		if ( theta < Contact::INHERITANCE_ANGULAR_THRESHOLD )
			continue;
		inheritance = true;
		break;
	}
	if (!inheritance)
		return NULL;
	return (*cp);
}


/*!
  Removes contact c from the body's contact list.  Assumes the contact is in the list.
  \sa addContact() 
 */
void
Body::removeContact(Contact *c)
{
  int i;
  std::list<Contact *>::iterator cp;

  setContactsChanged();

  if (showFC) {
    for (cp=contactList.begin(),i=0;cp!=contactList.end();cp++,i++)
      if (*cp == c) {
	    contactList.erase(cp);
	    break;
      }
    IVContactIndicators->removeChild(i);
  }
  else contactList.remove(c);

  delete c;
  numContacts--;
}

/*!
	Removes a contact c from the prevContactList
*/
void Body::removePrevContact(Contact *c)
{
	prevContactList.remove(c);
	c->setMate(NULL);
	delete c;
}

/*!
  Output method for writing body data to a text world configuration file.
*/
QTextStream&
operator<<(QTextStream &os, const Body &b)
{
  os << b.myFilename << endl;
  os << b.myWorld->getMaterialName(b.material) << endl;
  return os;
}


/*! Helper callback for generating list of body triangles */
void addTriangleCallBack(void* info, SoCallbackAction * action,
						 const SoPrimitiveVertex * v1, 
						 const SoPrimitiveVertex * v2, 
						 const SoPrimitiveVertex * v3)
{
	std::vector<Triangle> *triangles = (std::vector<Triangle>*) info;

	SbVec3f p1, p2, p3;
	SbMatrix mm = action->getModelMatrix();

	// Transform vertices (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	mm.multVecMatrix( v2->getPoint(), p2 );
	mm.multVecMatrix( v3->getPoint(), p3 );

	// Don't add degenerate triangles!
	if ((p1 == p2) || (p2 == p3) || (p1 == p3)) return;

	position nv1(p1[0], p1[1], p1[2]);
	position nv2(p2[0], p2[1], p2[2]);
	position nv3(p3[0], p3[1], p3[2]);
	Triangle newTri(nv1,nv2,nv3);
	triangles->push_back( newTri );
}

/*! Helper callback for generating list of body vertices */
void addVertexCallBack(void* info, SoCallbackAction * action, const SoPrimitiveVertex * v1)
{
	std::vector<position> *vertices = (std::vector<position>*) info;
	SbVec3f p1;
	SbMatrix mm = action->getModelMatrix();
	// Transform vertex (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	position nv1(p1[0], p1[1], p1[2]);
	vertices->push_back( nv1 );
}

/*! Helper callback for generating list of body vertices */
void addVerticesFromTriangleCallBack(void* info, SoCallbackAction * action,
									 const SoPrimitiveVertex * v1, 
									 const SoPrimitiveVertex * v2, 
									 const SoPrimitiveVertex * v3)
{
	std::vector<position> *vertices = (std::vector<position>*) info;
	SbVec3f p1, p2, p3;
	SbMatrix mm = action->getModelMatrix();
	// Transform vertices (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	mm.multVecMatrix( v2->getPoint(), p2 );
	mm.multVecMatrix( v3->getPoint(), p3 );
	vertices->push_back(position(p1[0], p1[1], p1[2]));
	vertices->push_back(position(p2[0], p2[1], p2[2]));
	vertices->push_back(position(p3[0], p3[1], p3[2]));
}

/*! Returns all the triangle that make up the geometry of this body */
void
Body::getGeometryTriangles(std::vector<Triangle> *triangles) const
{
	SoCallbackAction ca;
	ca.addTriangleCallback(SoShape::getClassTypeId(), addTriangleCallBack, triangles);
	ca.apply(getIVGeomRoot());
}

/*!	Returns all the vertices that make up the geometry of this body. 
	This function will return duplicates, as vertices are reported once 
	for each triangle that they are part of
*/
void
Body::getGeometryVertices(std::vector<position> *vertices) const
{
	SoCallbackAction ca;
	//unfortunately, this does not work as triangle vertices are not considered points by Coin
	//ca.addPointCallback(SoShape::getClassTypeId(), addVertexCallBack, vertices);
	//we have to use a triangle callback which leads to duplication
	ca.addTriangleCallback(SoShape::getClassTypeId(), addVerticesFromTriangleCallBack, vertices);
	ca.apply(getIVGeomRoot());
}

/*! Creates the geometry of the axes which show this body's local
	coordinate system. The axes are usually shown centered at the c.o.m
*/
void Body::createAxesGeometry()
{  
  IVWorstCase = new SoSeparator;  
  IVAxes = new SoSwitch;  
  if (graspItGUI) {
    SoSeparator *axesSep = new SoSeparator;
    axesTranToCOG = new SoTranslation;
    axesTranToCOG->translation.setValue(0,0,0);
    axesSep->addChild(axesTranToCOG);
    axesSep->addChild(IVWorstCase);
    
    axesScale = new SoScale;
	axesScale->scaleFactor = SbVec3f(1,1,1);
    axesSep->addChild(axesScale);
    axesSep->addChild(graspItGUI->getIVmgr()->getPointers()->getChild(2));
    IVAxes->addChild(axesSep);
  }
  if (!IVRoot) IVRoot = new SoSeparator;

  IVRoot->addChild(IVAxes);
}

///////////////////////////////////////////////////////////////////////////////
//                              DynamicBody
///////////////////////////////////////////////////////////////////////////////

/*!
  If there is a dynamic joint connected to this body, it is deleted before the
  body is destroyed.
*/
DynamicBody::~DynamicBody()
{
  if (dynJoint) delete dynJoint;
}

void
DynamicBody::init()
{
  maxRadius = 0.0; mass = 0.0;
  showAx = showDynCF = false;
  fixed = false; dynJoint=NULL; dynamicsComputedFlag = false;
  useDynamics = true;
  bbox_min = vec3(-1e+6,-1e+6,-1e+6);
  bbox_max = vec3(1e+6,1e+6,1e+6);
  CoG.set(0.0,0.0,0.0);
  for(int i=0; i<9; i++) {
	  I[i] = 0.0;
  }
  resetDynamics();
}

/*! Sets acceleration and velocity to 0 and the 7-dimensional
	state vector to match the current transform (which includes
	the position of the center of gravity.
*/
void
DynamicBody::resetDynamics()
{
  resetExtWrenchAcc();
  for (int i=0; i<6; i++) {
    a[i] = 0.0;
    v[i] = 0.0;
  }
  Quaternion quat = Tran.rotation();
  vec3 cogOffset = quat * (CoG-position::ORIGIN);
  q[0] = Tran.translation().x()+cogOffset.x();
  q[1] = Tran.translation().y()+cogOffset.y();
  q[2] = Tran.translation().z()+cogOffset.z();
  q[3] = quat.w;
  q[4] = quat.x;
  q[5] = quat.y;
  q[6] = quat.z;
}

/*! Constructs an empty DynamicBody.  Use load() to initialize this class
	from a model file, or cloneFrom to initialize from a different 
	dynamic body.
*/
DynamicBody::DynamicBody(World *w, const char *name) : Body(w,name)
{
	init();
}

/*! 
  Creates a dynamic body from the basic body \a b.  Computes the mass
  properties automatically assuming a mass of \a m and uniform mass
  distribution.
*/
DynamicBody::DynamicBody(const Body &b, double m) : Body(b)
{
  init();
  position defCoG;
  double defI[9];
  computeDefaultMassProp(defCoG, defI);
  setMass(m);
  setCoG(defCoG);
  setInertiaMatrix(defI);
  setMaxRadius(computeDefaultMaxRadius());
  //we need the effects of get tran on dynamics
  setTran(b.getTran());
}

/*! Clones another dynamic body; all dynamic paramters are copied
	over.
*/
void
DynamicBody::cloneFrom(const DynamicBody *original)
{
	Body::cloneFrom(original);
	setMass(original->mass);
	setCoG(original->CoG);
	setInertiaMatrix(original->I);
	setMaxRadius(original->maxRadius);
}

/*! Also looks for dynamic properties in the object file. If they are not 
	read from file, this will compute the default values from geometry and
	set them.
*/
int
DynamicBody::load(const QString &filename)
{
	if (Body::load(filename)!=SUCCESS)  return FAILURE;

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
	   QTWARNING("Could not open " + filename);
		return FAILURE;
	}
	QTextStream stream( &file );

	bool massInfo = false;
	bool overrideI = false;
	bool overrideCog = false;
	double loadI[9];
	position loadCog;
	double loadMass;
	QString line;
	while (nextCommentLine(&stream, &line)) {
		QString defStr = line.section('#',1).section(' ',0,0);
		QString valueStr;
		if (defStr=="mass")	{
			valueStr = line.section(' ',1,1);
			loadMass = valueStr.toDouble();
			if (loadMass <= 0) {
				QTWARNING("invalid mass in dynamic body file: " + myFilename);
				file.close();
				return FAILURE;
			}
			massInfo = true;
		}
		if (defStr=="cog") {
			double x,y,z;
			valueStr = line.section(' ',1);
			QTextStream(&valueStr,QIODevice::ReadOnly) >> x >> y >> z;
			loadCog = position(x,y,z);
			overrideCog = true;
		}
		if (defStr=="inertia_matrix") {
			line = stream.readLine().section('#',1);
			QTextStream(&line,QIODevice::ReadOnly) >> loadI[0] >> loadI[1] >> loadI[2];
			line = stream.readLine().section('#',1);    
			QTextStream(&line,QIODevice::ReadOnly) >> loadI[3] >> loadI[4] >> loadI[5];
			line = stream.readLine().section('#',1);
		    QTextStream(&line,QIODevice::ReadOnly) >> loadI[6] >> loadI[7] >> loadI[8];
			overrideI = true;
		}
	}
	file.close();      
	if (!massInfo) {
		loadMass = defaultMass;
		DBGA("Using default mass");
	}
	if (!overrideI || !overrideCog) {
		position defaultCog;
		double defaultI[9];
		computeDefaultMassProp(defaultCog, defaultI);
		if (!overrideI) {
			memcpy(loadI, defaultI, 9*sizeof(double));
			DBGA("Using default inertia matrix");
		}
		if (!overrideCog) {
			loadCog = defaultCog;
			DBGA("Using default center of gravity");
		}
	}
	setMass(loadMass);
	setCoG(loadCog);
	setInertiaMatrix(loadI);
	setMaxRadius(computeDefaultMaxRadius());
	return SUCCESS;
}

/*! Also has to reset dynamics since the current state has to be changed
	to match the new CoG.
*/
void
DynamicBody::setCoG(const position &newCoG)
{
	CoG = newCoG;  
	resetDynamics();
	//axesTranToCOG->translation.setValue(CoG.x(), CoG.y(), CoG.z());
	axesTranToCOG->translation.setValue(0,0,0);
}

/*! The max radius can be thought of as the largest distance from the center
	of gravity to the edge of the object.
*/
void
DynamicBody::setMaxRadius(double maxRad)
{
	maxRadius = maxRad;
	axesScale->scaleFactor = SbVec3f(maxRadius / AXES_SCALE, maxRadius / AXES_SCALE, maxRadius / AXES_SCALE);
}

void
DynamicBody::setInertiaMatrix(const double *newI)
{
	memcpy(I, newI, 9*sizeof(double));
}

double
DynamicBody::computeDefaultMaxRadius()
{
	std::vector<position> vertices;
	getGeometryVertices(&vertices);
	if (vertices.empty()) {
		DBGA("No vertices found when computing maxRadius!");
	}
	double maxRad = 0.0;
	for (int i=0; i<(int)vertices.size(); i++) {
		double tmpRadius = (CoG - vertices[i]).len();
		if (tmpRadius > maxRad) maxRad = tmpRadius;
	}
	return maxRad;
}

/*!
  Support routine for computing body mass properties.  This code is
  adapted from code written by Brian Mirtich.
*/
void
DynamicBody::compProjectionIntegrals(FACE &f, int A, int B)
{
  double a0, a1, da;
  double b0, b1, db;
  double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
  double a1_2, a1_3, b1_2, b1_3;
  double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
  double Cab, Kab, Caab, Kaab, Cabb, Kabb;
  int i;

  P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

  for (i = 0; i < 3; i++) {
    a0 = f.verts[i][A];
    b0 = f.verts[i][B];
    a1 = f.verts[(i+1) % 3][A];
    b1 = f.verts[(i+1) % 3][B];
    da = a1 - a0;
    db = b1 - b0;
    a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
    b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
    a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
    b1_2 = b1 * b1; b1_3 = b1_2 * b1;

    C1 = a1 + a0;
    Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
    Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
    Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
    Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
    Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
    Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

    P1 += db*C1;
    Pa += db*Ca;
    Paa += db*Caa;
    Paaa += db*Caaa;
    Pb += da*Cb;
    Pbb += da*Cbb;
    Pbbb += da*Cbbb;
    Pab += db*(b1*Cab + b0*Kab);
    Paab += db*(b1*Caab + b0*Kaab);
    Pabb += da*(a1*Cabb + a0*Kabb);
  }

  P1 /= 2.0;
  Pa /= 6.0;
  Paa /= 12.0;
  Paaa /= 20.0;
  Pb /= -6.0;
  Pbb /= -12.0;
  Pbbb /= -20.0;
  Pab /= 24.0;
  Paab /= 60.0;
  Pabb /= -60.0;
}

/*!
  Support routine for computing body mass properties.  This code is
  adapted from code written by Brian Mirtich.
*/
void
DynamicBody::compFaceIntegrals(FACE &f,int A, int B, int C)
{
  double *n, w;
  double k1, k2, k3, k4;

  compProjectionIntegrals(f,A,B);

  w = f.w;
  n = f.norm;
  k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

  Fa = k1 * Pa;
  Fb = k1 * Pb;
  Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

  Faa = k1 * Paa;
  Fbb = k1 * Pbb;
  Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
	 + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faaa = k1 * Paaa;
  Fbbb = k1 * Pbbb;
  Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab 
	   + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
	   + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
	   + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faab = k1 * Paab;
  Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
  Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
	 + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
}

/*!
  Given a list of the body vertices, and the number of triangles, this
  computes the center of gravity and inertia matrix by assuming a uniform
  mass distribution.  This code is based on code written by Brian Mirtich.
*/
void
DynamicBody::computeDefaultMassProp(position &defaultCoG, double *defaultI)
{
  FACE f;
  double dx1,dy1,dz1,dx2,dy2,dz2;
  double nx, ny, nz,len;
  int i,A,B,C;
  double r[3], v1[3], v2[3], v3[3];
  double density;

  std::vector<Triangle> triangles;
  getGeometryTriangles(&triangles);
  
  f.verts[0] = v1;
  f.verts[1] = v2;
  f.verts[2] = v3;

	// volume integrals
  double T0, T1[3], T2[3], TP[3];

  T0 = T1[0] = T1[1] = T1[2] 
     = T2[0] = T2[1] = T2[2] 
     = TP[0] = TP[1] = TP[2] = 0;

  if (triangles.empty()) {
	 DBGA("No triangles found when computing mass properties!");
	 return;
  }

  for (i = 0; i < (int)triangles.size(); i++) {
	triangles[i].v1.get(v1);
	triangles[i].v2.get(v2);
	triangles[i].v3.get(v3);

	dx1 = f.verts[1][0] - f.verts[0][0];
    dy1 = f.verts[1][1] - f.verts[0][1];
    dz1 = f.verts[1][2] - f.verts[0][2];
    dx2 = f.verts[2][0] - f.verts[1][0];
    dy2 = f.verts[2][1] - f.verts[1][1];
    dz2 = f.verts[2][2] - f.verts[1][2];
    nx = dy1 * dz2 - dy2 * dz1;
    ny = dz1 * dx2 - dz2 * dx1;
    nz = dx1 * dy2 - dx2 * dy1;
    len = sqrt(nx * nx + ny * ny + nz * nz);

    f.norm[0] = nx / len;
    f.norm[1] = ny / len;
    f.norm[2] = nz / len;
    f.w = - f.norm[0] * f.verts[0][0]
           - f.norm[1] * f.verts[0][1]
           - f.norm[2] * f.verts[0][2];

    nx = fabs(f.norm[0]);
    ny = fabs(f.norm[1]);
    nz = fabs(f.norm[2]);
    if (nx > ny && nx > nz) C = 0;
    else C = (ny > nz) ? 1 : 2;
    A = (C + 1) % 3;
    B = (A + 1) % 3;

    compFaceIntegrals(f,A,B,C);

    T0 += f.norm[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

    T1[A] += f.norm[A] * Faa;
    T1[B] += f.norm[B] * Fbb;
    T1[C] += f.norm[C] * Fcc;
    T2[A] += f.norm[A] * Faaa;
    T2[B] += f.norm[B] * Fbbb;
    T2[C] += f.norm[C] * Fccc;
    TP[A] += f.norm[A] * Faab;
    TP[B] += f.norm[B] * Fbbc;
    TP[C] += f.norm[C] * Fcca;
  }

  T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
  T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
  TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;
#ifdef GRASPITDBG
  printf("\nT1 =   %+20.6f\n\n", T0);

  printf("Tx =   %+20.6f\n", T1[0]);
  printf("Ty =   %+20.6f\n", T1[1]);
  printf("Tz =   %+20.6f\n\n", T1[2]);
  
  printf("Txx =  %+20.6f\n", T2[0]);
  printf("Tyy =  %+20.6f\n", T2[1]);
  printf("Tzz =  %+20.6f\n\n", T2[2]);

  printf("Txy =  %+20.6f\n", TP[0]);
  printf("Tyz =  %+20.6f\n", TP[1]);
  printf("Tzx =  %+20.6f\n\n", TP[2]);
#endif

  //assume unity mass
  density = 1.0 / T0;

  /* compute center of mass */
  r[0] = T1[0] / T0;
  r[1] = T1[1] / T0;
  r[2] = T1[2] / T0;
  defaultCoG.set(r);

  /* compute inertia tensor */
  defaultI[0] = density * (T2[1] + T2[2]);
  defaultI[4] = density * (T2[2] + T2[0]);
  defaultI[8] = density * (T2[0] + T2[1]);
  defaultI[1] = defaultI[3] = - density * TP[0];
  defaultI[5] = defaultI[7] = - density * TP[1];
  defaultI[6] = defaultI[2] = - density * TP[2];

  /* translate inertia tensor to center of mass */
  defaultI[0] -= (r[1]*r[1] + r[2]*r[2]);
  defaultI[4] -= (r[2]*r[2] + r[0]*r[0]);
  defaultI[8] -= (r[0]*r[0] + r[1]*r[1]);
  defaultI[1] = defaultI[3] += r[0] * r[1]; 
  defaultI[5] = defaultI[7] += r[1] * r[2]; 
  defaultI[6] = defaultI[2] += r[2] * r[0]; 

#ifdef GRASPITDBG
  printf("Name: %s\n",getName().latin1());
  printf("center of mass:  (%+12.6f,%+12.6f,%+12.6f)\n\n", r[0], r[1], r[2]);

  printf("inertia tensor with origin at c.o.m. (scaled for unity mass):\n");
  printf("%+15.6f  %+15.6f  %+15.6f\n", defaultI[0], defaultI[1], defaultI[2]);
  printf("%+15.6f  %+15.6f  %+15.6f\n", defaultI[3], defaultI[4], defaultI[5]);
  printf("%+15.6f  %+15.6f  %+15.6f\n\n", defaultI[6], defaultI[7], defaultI[8]);
#endif
}

void 
DynamicBody::setDefaultViewingParameters()
{
	Body::setDefaultViewingParameters();
	showAx = false;
}

/*!
  Changes the state of the Inventor switch node controlling whether the
  coordinate axes on the body are visible
*/
void
DynamicBody::showAxes(bool on)
{
	DBGA("Show axes: " << on);
  if (on) IVAxes->whichChild = 0;
  else IVAxes->whichChild = -1;
  showAx = on;
}

/*!
  Sets whether dynamic contact forces should be drawn during dynamic
  simulation.
*/
void
DynamicBody::showDynContactForces(bool on)
{
  showDynCF = on;
}


/*!
  Resets the external wrench accumulator to 0.
*/
void
DynamicBody::resetExtWrenchAcc()
{
  extWrenchAcc[0] = extWrenchAcc[1] = extWrenchAcc[2] = 0.0;
  extWrenchAcc[3] = extWrenchAcc[4] = extWrenchAcc[5] = 0.0;
}

/*!
  Adds \a extW to the external wrench accumulator.
*/
void
DynamicBody::addExtWrench(double *extW){
  extWrenchAcc[0] += extW[0];
  extWrenchAcc[1] += extW[1];
  extWrenchAcc[2] += extW[2];
  extWrenchAcc[3] += extW[3];
  extWrenchAcc[4] += extW[4];
  extWrenchAcc[5] += extW[5];
}

/*! 
   Adds a force expressed in world coordinates, to the external force
   accumulator for this body.
*/
void
DynamicBody::addForce(vec3 force)
{
  extWrenchAcc[0] += force[0];
  extWrenchAcc[1] += force[1];
  extWrenchAcc[2] += force[2];
}

/*!
  Adds a torque expressed in world coordinates, to the external force
  accumulator for this body.
 */
void
DynamicBody::addTorque(vec3 torque)
{
  extWrenchAcc[3] += torque[0];
  extWrenchAcc[4] += torque[1];
  extWrenchAcc[5] += torque[2];
  DBGP("Adding torque "<< torque);
}

/*
  Adds a torque expressed in body coordinates, to the external force
  accumulator for this body.
*/
void
DynamicBody::addRelTorque(vec3 torque)
{
  vec3 worldTorque;
  DBGP("Adding rel torque "<< torque);
  worldTorque = Tran.rotation() * torque;
  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];
  DBGP("     world torque "<< worldTorque);
}


/*! 
   Adds a force expressed in world coordinates at a position also expressed
   world coordinates. This force and the computed torque are added to the
   external force accumulator for this body.
*/
void
DynamicBody::addForceAtPos(vec3 force,position pos)
{
  vec3 worldTorque;
  vec3 worldPos;
  worldPos = (pos - CoG * Tran);

  worldTorque = worldPos * force;
  extWrenchAcc[0] += force[0];
  extWrenchAcc[1] += force[1];
  extWrenchAcc[2] += force[2];

  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];
}


/*!
  Adds a force expressed in body coordinates at a position also expressed
  body coordinates. This force and the computed torque are added to the
  external force accumulator for this body.
*/
void
DynamicBody::addForceAtRelPos(vec3 force,position pos)
{
  vec3 worldForce;
  vec3 worldTorque;

  worldForce = Tran.rotation() * force;
  worldTorque = Tran.rotation() * ((pos - CoG) * force);
  extWrenchAcc[0] += worldForce[0];
  extWrenchAcc[1] += worldForce[1];
  extWrenchAcc[2] += worldForce[2];

  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];

  DBGP("Adding rel force "<< force << " at " << pos);
  DBGP("    world torque "<< worldTorque << " worldForce " << worldForce);
}

// Let the dynamics routine take care of breaking the contacts
bool 
DynamicBody::setPos(const double *new_q)
{
	double norm;
	// is the object within its permissable area?
	if (new_q[0] < bbox_min.x() || new_q[0] > bbox_max.x()) return false;
	if (new_q[1] < bbox_min.y() || new_q[1] > bbox_max.y()) return false;
	if (new_q[2] < bbox_min.z() || new_q[2] > bbox_max.z()) return false;
  
	memcpy(q,new_q,7*sizeof(double));

	// normalize the quaternion
	norm = sqrt(q[3]*q[3]+q[4]*q[4]+q[5]*q[5]+q[6]*q[6]);
	q[3] /= norm;
	q[4] /= norm;
	q[5] /= norm;
	q[6] /= norm;

	Quaternion rot(q[3],q[4],q[5],q[6]);
	vec3 cogOffset = rot * (CoG-position::ORIGIN);
	transf tr = transf(rot,vec3(q[0],q[1],q[2])-cogOffset);

	Tran = tr;
	Tran.toSoTransform(IVTran);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);

	return true;
}

/*!
  Save the current dynamic state of the body.
  May soon be replaced by pushState().
*/
void
DynamicBody::markState()
{
  memcpy(markedQ,q,7*sizeof(double));
  memcpy(markedV,v,6*sizeof(double));
}

/*!
  Restore the marked dynamic state of the body.
  May soon be replaced by popState().
*/
void
DynamicBody::returnToMarkedState()
{
  memcpy(v,markedV,6*sizeof(double));
  setPos(markedQ);
}

/*! 
  Push the current dynamic state of the body onto a local stack.
*/
void
DynamicBody::pushState()
{
  double *tmp = new double[7];
  memcpy(tmp,q,7*sizeof(double));
  qStack.push_back(tmp);

  tmp = new double[6];
  memcpy(tmp,v,6*sizeof(double));
  vStack.push_back(tmp);  
}

/*!
  Pop the current dynamic state of the body from a local stack.
*/
bool
DynamicBody::popState()
{
	if (qStack.empty()) return false;
	memcpy(v,vStack.back(),6*sizeof(double));
	setPos(qStack.back());

	// don't pop off the first saved state.
	if (++qStack.begin() != qStack.end()) {
		delete [] vStack.back();
		delete [] qStack.back();
		vStack.pop_back(); 
		qStack.pop_back();
		return true;
	} else {
		return false;
    }
}

void
DynamicBody::clearState()
{
	//we don't actually change the state of the object, just clear
	//the stack. For some reason we leave the last state in.
	if (qStack.empty()) return;
	while (++qStack.begin() != qStack.end()) {
		delete [] vStack.back();
		delete [] qStack.back();
		vStack.pop_back(); 
		qStack.pop_back();
	}
}
/*!
  Calls Body::setTran then updates the dynamic state of the body.
*/
int
DynamicBody::setTran(transf const& tr)
{
  if (tr == Tran) return SUCCESS;
  if (Body::setTran(tr) == FAILURE) return FAILURE;
  Quaternion quat = Tran.rotation();
  vec3 cogOffset = quat * (CoG-position::ORIGIN);
  q[0] = Tran.translation().x()+cogOffset.x();
  q[1] = Tran.translation().y()+cogOffset.y();
  q[2] = Tran.translation().z()+cogOffset.z();
  q[3] = quat.w;
  q[4] = quat.x;
  q[5] = quat.y;
  q[6] = quat.z;  
  if (fixed) fix();
  return SUCCESS;
}

/*!
  Fixes the body so that it does not move during dynamic simulation.  This
  is done by addind a fixed dynamic joint that will constrain all 6 body
  velocities to 0.
*/
void
DynamicBody::fix()
{
  fixed = true;
  setDynJoint(new FixedDynJoint(NULL,this,Tran));
}

/*!
  Removes the fixed dynamic joint so the body is free to move again during
  dynamic simulation.
*/
void
DynamicBody::unfix()
{
  fixed = false;
  setDynJoint(NULL);
}

/*!
  Sets the dynamic joint connected to this body to \a dj.
*/
void
DynamicBody::setDynJoint(DynJoint *dj)
{
  if (dynJoint) delete dynJoint;
  dynJoint = dj;
}

///////////////////////////////////////////////////////////////////////////////
//                            Link
///////////////////////////////////////////////////////////////////////////////

/*
  Initializes a robot link.  \a r is the pointer to the robot that owns
  this link, \a c is the index of the chain this link is in, and \a l is
  the link number of this link within that chain.
*/
Link::Link(Robot *r,int c, int l,World *w,const char *name) : DynamicBody(w,name)
{
  owner = r; chainNum = c; linkNum = l; showVC = false; showFC = false;
}


/*
  Stub destructor.
*/
Link::~Link()
{
}


/*!
  Sets BOTH the worldElement's AND the robot's \a contactsChanged flag.
*/
void
Link::setContactsChanged()
{
  WorldElement::setContactsChanged();
  owner->setContactsChanged();
}
/*!
	This acts like contactPreventMotion, except in the case of a link belonging to a robot.
	In this case, contact against another link of the same robot does not prevent motion,
	as we assume the entire robot is moving.
*/
bool
Link::externalContactsPreventMotion(const transf& motion)
{
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;

	contactList = getContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		if ( (*cp)->getBody2()->getOwner() == getOwner() )
			continue;
		if ((*cp)->preventsMotion(motion)) {
			return true;
		}
	}
  return false;
}

/*! By convention, in GraspIt! each link's origin is located at the next joint in the chain */
position
Link::getDistalJointLocation()
{
	return position(0,0,0);
}

/*! The translation component of the previous joint transform gives me the location of that joint
	in this joint's coordinate system */
position
Link::getProximalJointLocation()
{
	int jointNum = owner->getChain(chainNum)->getLastJoint(linkNum);
	Joint *j = owner->getChain(chainNum)->getJoint(jointNum);
	vec3 p = j->getTran().inverse().translation();
	return position( p.x(), p.y(), p.z() );
}

vec3
Link::getProximalJointAxis()
{
	int jointNum = owner->getChain(chainNum)->getLastJoint(linkNum);
	Joint *j = owner->getChain(chainNum)->getJoint(jointNum);
	vec3 r = vec3(0,0,1) * j->getTran().inverse();
	return r;
}

///////////////////////////////////////////////////////////////////////////////
//                            GraspableBody
///////////////////////////////////////////////////////////////////////////////

/*
  Stub constructor.
*/
GraspableBody::GraspableBody(World *w,const char *name) : DynamicBody(w,name)
{
}

/*
  Stub destructor.
*/
GraspableBody::~GraspableBody()
{

}

/*! Shows friction cones and axes, and sets transparency to 0.4 */
void 
GraspableBody::setDefaultViewingParameters()
{
	showFC = true;
	showVC = true;
	showAxes(true);
	setTransparency(0.4f);
}

/*! Probably not needed, just calls super*/
void
GraspableBody::cloneFrom(const GraspableBody *original) 
{
	DynamicBody::cloneFrom(original);
	setDefaultViewingParameters();
}

/*!
  Output method for writing body data to a text world configuration file
*/
QTextStream&
operator<<(QTextStream &os, const GraspableBody &gb)
{
  os << gb.myFilename << endl;
  return os;
}


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
// $Id: grasp.cpp,v 1.39.2.1 2009/04/24 22:19:20 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the grasp class, which analyzes grasps
 */

/* standard C includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <exception>
#include <typeinfo>

#include <QString>
#include <q3listbox.h>

//#include "mainWindow.h"
#include "grasp.h"
#include "world.h"
#include "robot.h"
#include "joint.h"
#include "body.h"
#include "contact.h"
#include "gws.h"
#include "quality.h"
#include "gwsprojection.h"
#include "matrix.h"
#include "matvec3D.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

#include "maxdet.h"

#include <Inventor/Qt/SoQtComponent.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define G_ 9.81

typedef float * floatP;

bool errorOccurred = false;

double Grasp::GFO_WEIGHT_FACTOR = 1.0;

//print out error message and exit
void FatalErrMsg(char* s)
{
  fprintf(stderr,"error: %s\n",s);
  exit(-1);
}

#define errMsg(S_)                 \
{                                  \
  fprintf(stderr,"error: %s\n",S_);\
  errorOccurred = true;            \
  return;                          \
} 

  

/*!
  Create a diagonal-block identity matrix
*/
double * block_Identity(int numBlocks, int * blockSizes, int pkSize)
{

  int i, j, itemIndex, blockDim;
  double db0=0.0,*identity= (double*) malloc(pkSize*sizeof(double));
  
  dcopy(pkSize,&db0, 0, identity, 1);
  itemIndex=0;
  for(i=0;i<numBlocks; i++){
    blockDim=blockSizes[i];
    for(j=0;j<blockDim;j++){
      identity[itemIndex]=1.0;
      itemIndex+=blockDim-j;
    }
  }
  return(identity);
}

/*!
  Initialize grasp object variables.  A grasp should only be created by a
  hand constructor, which passes a pointer to itself.
*/
Grasp::Grasp(Hand *h)
{
  int i;

  hand = h;
  object = NULL;
  numContacts=0;
  valid=true;
  useGravity = false;
  pRstFile = NULL;

  /* initialize all of the force optimization variables */
  graspMap = Jacobian = externalTorques = nullSpace = optmx0 = minNormSln=
    optTorques = NULL;
  F_blkszs = G_blkszs = NULL;
  F = G = W = Z = c = NULL;
  nullDim = L = K = 0;

  gamma = 100.0;  // temporary
  abstol = 0.05;  // temporary
  reltol = 0.0065; // temporary
  negativeFlag = 1;
  graspCounter = 0;

  initz0 = optmz0 = extendOptmz0 = NULL;
  feasZHistory = feasXHistory = optmZHistory = optmXHistory = NULL;
  extendOptmZHistory = NULL;

  for (i=0;i<6;i++) minWrench[i]=0.0;

  numQM = 0;
}

/*!
  Deletes all quality measures, grasps wrench spaces, and projections
  associated with this grasp.  Frees all of the dynamically allocated
  vectors and matrics used for GFO.
 */
Grasp::~Grasp()
{
  int i;
  std::list<GWSprojection *>::iterator gpp;
  std::list<GWS *>::iterator gp;

  std::cout << "Deleting grasp" <<std::endl;

  if (graspMap)          delete [] graspMap;
  if (nullSpace)         delete [] nullSpace;
  if (optmx0)            delete [] optmx0;
  if (minNormSln)        delete [] minNormSln;
  if (Jacobian)          delete [] Jacobian;
  if (externalTorques)   delete [] externalTorques;
  if (F_blkszs)          delete [] F_blkszs;
  if (G_blkszs)          delete [] G_blkszs;
  if (F)                 delete [] F;
  if (G)                 delete [] G;
  if (Z)                 delete [] Z;
  if (W)                 delete [] W;
  if (c)                 delete [] c;
  if (initz0)            delete [] initz0;
  if (optmz0)            delete [] optmz0;
  if (extendOptmz0)      delete [] extendOptmz0;
  if (feasZHistory)      delete [] feasZHistory;
  if (optmZHistory)      delete [] optmZHistory;
  if (extendOptmZHistory)delete [] extendOptmZHistory;
  if (feasXHistory)      delete [] feasXHistory;
  if (optmXHistory)      delete [] optmXHistory;

  for (i=0;i<numQM;i++) removeQM(0);
  for (gpp=projectionList.begin();gpp!=projectionList.end();gpp++)
    delete *gpp;
  for (gp=gwsList.begin();gp!=gwsList.end();gp++)
    delete *gp;
}


/*!
  Creates a new GWS of type \a type and adds it to the list of GWS's associated
  with this grasp if it does not exist already.  Current possible types of
  GWS's are "L1 Norm", and "LInfinity Norm".  Returns a pointer to a gws
  of the requested type that was either created or found.
*/
GWS *
Grasp::addGWS(const char *type)
{
  GWS *gws = NULL;
  std::list<GWS *>::iterator gp;

  //first check if we have the needed GWS already
  for (gp=gwsList.begin();gp!=gwsList.end();gp++)
    if (!strcmp((*gp)->getType(),type))
      gws = *gp;
    
  if (!gws) {
    gws = GWS::createInstance(type,this);
    
    gwsList.push_back(gws);
    printf("created new %s GWS.\n",type);
  }

  gws->ref();  // add 1 to the reference count
    
  return gws;
}

/*!
  Given a pointer to the GWS to remove, this decrements the reference count
  for that GWS.  If the reference count becomes 0, the GWS is deleted.
*/
void
Grasp::removeGWS(GWS *gws)
{
  DBGP("removing gws");
  gws->unref();
  if (gws->getRefCount() == 0) {
    DBGP("deleting gws");
    gwsList.remove(gws);
    delete gws;
  } else {
	  DBGP("gws refcount: "<<gws->getRefCount());
  }
}

/*!
  Adds the quality measure to the grasp's list of quality measures.
*/
void
Grasp::addQM(QualityMeasure *qm)
{
   qmList.push_back(qm);
   numQM++;
}

/*!
  Replaces an existing quality measure with a new one and deletes the old one.
  \a which selects which qm to replace where the index of the first qm in the
  list is 0.
*/
void
Grasp::replaceQM(int which,QualityMeasure *qm)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  qmList.insert(qp,qm);

  if (qp!=qmList.end()) {
    delete *qp;
    qmList.erase(qp);
  }
}

/*!
  Returns a pointer to the requested quality measure.  \a which is the index
  of the qm to return, where 0 is the first qm in the list.
*/
QualityMeasure *
Grasp::getQM(int which)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  if (qp!=qmList.end()) return *qp;
  return NULL;
}

/*!
  Removes a quality measure from the list and deletes it.  \a which is the
  index of the qm to remove, where 0 is the first qm in the list.
*/
void
Grasp::removeQM(int which)
{
  int i;
  std::list<QualityMeasure *>::iterator qp;
  
  for (qp=qmList.begin(),i=0; qp!=qmList.end() && i!=which; qp++,i++);
  if (qp!=qmList.end()) {
    printf("Removing QM\n");
    delete *qp;
    qmList.erase(qp);
    numQM--;
  }
}

/*!
  Add a grasp wrench space projection to the grasp's list of projections.
*/
void
Grasp::addProjection(GWSprojection *gp)
{
  projectionList.push_back(gp);
  update();
}

/*!
  Removes a grasp wrench space projection from the list and deletes it.
*/
void
Grasp::removeProjection(GWSprojection *gp)
{
    projectionList.remove(gp);
    delete gp;
}

/*!
  This is the projection window close callback function.  When the projection
  window is closed, this removes the associated GWSprojection.
*/
void
Grasp::destroyProjection(void * user, SoQtComponent *)
{
  GWSprojection *gp = (GWSprojection *)user;
  gp->getGWS()->getGrasp()->removeProjection(gp);
}

/*!
  Updates the grasp by collecting all the current contacts on the hand and
  rebuilding any grasp wrench spaces and their projections.  This is usually
  called after the contacts on the hand have changed.
*/
void
Grasp::update()
{
  QString valStr;
  hand->resetContactsChanged();
  if (object) {
	  //fprintf(stderr,"Collecting real contacts\n");
	  collectContacts();
  } else {
	  //fprintf(stderr,"Collecting virtual contacts\n");
	  collectVirtualContacts();
  }
  DBGP("\n\nAnalyzing grasp...");
  DBGP("numContacts: " << numContacts);	
  updateWrenchSpaces();
}

void
Grasp::updateWrenchSpaces3D()
{
	std::list<GWS *>::iterator gp; 
	// rebuild grasp wrench spaces in 3D
	for (gp=gwsList.begin();gp!=gwsList.end();gp++) {
		(*gp)->build3D();
	}
}

void
Grasp::updateWrenchSpaces()
{
  std::list<GWS *>::iterator gp; 
  std::list<GWSprojection *>::iterator pp;  

  //for tests with the online planner
  vec3 gravDirection(0,0,1);
  
  //SCALE gravity to some arbitrary value; here to 0.5 of what one contact can apply
  gravDirection = 0.5 * gravDirection;

  //compute the direction of world gravity forces relative to the object
  if (useGravity && object) {
	  gravDirection = gravDirection * object->getTran().inverse();
  }

  // rebuild grasp wrench spaces
  for (gp=gwsList.begin();gp!=gwsList.end();gp++) {
	  if (useGravity && object) {
		  (*gp)->setGravity(true, gravDirection);
	  } else {
		  (*gp)->setGravity(false);
	  }
      (*gp)->build();
  }

  // update the GWS projections
  for (pp=projectionList.begin();pp!=projectionList.end();pp++) {
    (*pp)->update();
  }

}

/*!
  Gathers the contacts on all links of the hand that are mated with contacts
  on the grasped object and adds them to the contactVec .  
*/
void
Grasp::collectContacts()
{
  int f,l;
  std::list<Contact *>::iterator cp;
  std::list<Contact *> contactList;

  contactVec.clear();
  numContacts = 0;

  contactList = hand->getPalm()->getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++)
    if ((*cp)->getBody2()==object) {
	  (*cp)->getMate()->computeWrenches();
      contactVec.push_back(*cp);
      numContacts++;
    }

  for(f=0;f<hand->getNumFingers();f++) {
    for (l=0;l<hand->getFinger(f)->getNumLinks();l++) {
      contactList = hand->getFinger(f)->getLink(l)->getContacts();
      for (cp=contactList.begin();cp!=contactList.end();cp++){
		if ((*cp)->getBody2()==object) {
		  (*cp)->getMate()->computeWrenches();
		  contactVec.push_back(*cp);
		  numContacts++;
		}
	  }
	}
  } 
  DBGP("Contacts: " << numContacts);
}

vec3 Grasp::virtualCentroid()
{
	vec3 cog(0,0,0);
	position pos;

	/*
	//COG AS CENTROID
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		cog = cog + vec3( pos.toSbVec3f() );
	}
	cog = ( 1.0 / (int)contactVec.size() ) * cog;
	//fprintf(stderr,"CoG: %f %f %f\n",cog.x(), cog.y(), cog.z());
	*/

	//COG as center of bounding box
	position topCorner(-1.0e5, -1.0e5, -1.0e5), bottomCorner(1.0e5, 1.0e5, 1.0e5);
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		if ( pos.x() > topCorner.x() ) topCorner.x() = pos.x();
		if ( pos.y() > topCorner.y() ) topCorner.y() = pos.y();
		if ( pos.z() > topCorner.z() ) topCorner.z() = pos.z();
		if ( pos.x() < bottomCorner.x() ) bottomCorner.x() = pos.x();
		if ( pos.y() < bottomCorner.y() ) bottomCorner.y() = pos.y();
		if ( pos.z() < bottomCorner.z() ) bottomCorner.z() = pos.z();
	}
	cog = 0.5 * (topCorner - bottomCorner);
	cog = vec3(bottomCorner.toSbVec3f()) + cog;

	return cog;
}

/*! Assumes all the contacts in the list are virtual, and computes the centroid
	and max radius for the grasp based on these virtual contacts.
*/
void
Grasp::setVirtualCentroid()
{
	vec3 cog = virtualCentroid();
	position pos;

	//VARIABLE RADIUS relative to cog location
	vec3 radius;
	double maxRadius = 0;
	for (int i=0; i<(int)contactVec.size(); i++) {
		pos = ((VirtualContact*)contactVec[i])->getWorldLocation();
		radius =  vec3( pos.toSbVec3f() ) - cog;
		if ( radius.len() > maxRadius) maxRadius = radius.len();
	}

	//FIXED radius to allow better inter-grasp comparison (exact value pulled out of thin air)
	maxRadius = 150;

	//fprintf(stderr,"Max radius: %f\n",maxRadius);

	for (int i=0; i<(int)contactVec.size(); i++) {
		((VirtualContact*)contactVec[i])->setCenter( position(cog.toSbVec3f()) );
		((VirtualContact*)contactVec[i])->setRadius(maxRadius);
		//((VirtualContact*)contactVec[i])->getWorldIndicator();
	}
}

/*!	Assumes that all contacts are virtual, but we do have an object so we 
	set its centroid  (IN WORLD COORDINATES) and maxradius to all virtual 
	contacts.
*/
void
Grasp::setRealCentroid(GraspableBody *body)
{
	position cog = body->getCoG() * body->getTran();
	double maxRadius = body->getMaxRadius();
	for (int i=0; i<(int)contactVec.size(); i++) {
		((VirtualContact*)contactVec[i])->setCenter(cog);
		((VirtualContact*)contactVec[i])->setRadius(maxRadius);
	}
}

/*!  Gathers the virtual contacts on all links of the hand and adds them to 
	the internal list in contactVec. Any GWS computations should then be 
	able to proceed regardless of the fact that these are virtual contacts.
	However, since we might not have an object, information about the 
	centroid to be used as reference point, and the max radius used for
	converting torques, are also computed and stored in the virtual
	contacts themselves.
*/
void
Grasp::collectVirtualContacts()
{
	int f,l;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;

	contactVec.clear();
	numContacts = 0;

	contactList = hand->getPalm()->getVirtualContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		contactVec.push_back(*cp);
		numContacts++;
	}

	for(f=0;f<hand->getNumFingers();f++) {
		for (l=0;l<hand->getFinger(f)->getNumLinks();l++) {
			contactList = hand->getFinger(f)->getLink(l)->getVirtualContacts();
			for (cp=contactList.begin();cp!=contactList.end();cp++){
				contactVec.push_back(*cp);
				numContacts++;
			}
		}
	}

	if (object == NULL) {
		setVirtualCentroid();
		for (int i=0; i<(int)contactVec.size(); i++) {
			((VirtualContact*)contactVec[i])->computeWrenches(false);
		}
	} else {
		setRealCentroid(object);
		//for (int i=0; i<(int)contactVec.size(); i++) {
			//((VirtualContact*)contactVec[i])->setObject(object);
			//((VirtualContact*)contactVec[i])->computeWrenches(true);
			//((VirtualContact*)contactVec[i])->getWorldIndicator(true);
		//}
	}
 }

double
Grasp::getMaxRadius()
{
	if (object) return object->getMaxRadius();
	if (numContacts == 0) return 0;
	return ((VirtualContact*)contactVec[0])->getMaxRadius();
}

position
Grasp::getCoG()
{
	if (object) return object->getCoG();
	if (numContacts == 0) return position(0,0,0);
	return ((VirtualContact*)contactVec[0])->getCenter();
}

/*!
  Computes the jacobian for the base frame of link \a l on finger \a f
  relative to the base frame of the finger. This is wrt THE FRAME OF LINK!!!
  
  Distances USED TO BE converted to meters (check why this is...). Now they are
  kept in MILLIMETERS!
*/
double *
Grasp::getLinkJacobian(int f, int l)
{
  int j,col;
  Joint *jointPtr;
  int numDOF = hand->getNumDOF();
  double *jac = new double[6*numDOF];
  double k;
  mat3 m;
  vec3 p;
  transf T;
  double db0 = 0.0;

  dcopy(6*numDOF,&db0,0,jac,1);
  
  //I use f=-1 on virtual contacts to show that a contact is on the palm
  if (f < 0) return jac;

  for (j=hand->getFinger(f)->getLastJoint(l);j>=0;j--) {
    jointPtr = hand->getFinger(f)->getJoint(j);
    col = jointPtr->getDOFNum();
    
	k = hand->getDOF(jointPtr->getDOFNum())->getStaticRatio(jointPtr);
	T = T * jointPtr->getDH()->getTran();
    m = T.affine();
    p = T.translation();
    
    if (jointPtr->getType() == REVOLUTE) {
      jac[col*6]   += k*(-m.element(0,0)*p.y() + m.element(0,1)*p.x());
      jac[col*6+1] += k*(-m.element(1,0)*p.y() + m.element(1,1)*p.x());
      jac[col*6+2] += k*(-m.element(2,0)*p.y() + m.element(2,1)*p.x());
      jac[col*6+3] += k*m.element(0,2);
      jac[col*6+4] += k*m.element(1,2);
      jac[col*6+5] += k*m.element(2,2);
    } else {
      jac[col*6]   += k*m.element(0,2);
      jac[col*6+1] += k*m.element(1,2);
      jac[col*6+2] += k*m.element(2,2);
      jac[col*6+3] += 0.0;
      jac[col*6+4] += 0.0;
      jac[col*6+5] += 0.0;
    }
  }
  return jac;
}

/*!
  Constructs the hand jacobian.  It currently assumes there is no palm motion,
  hence the matrix is \c numDOF x \c numWrenches (\c numWrenches = 3 *
  \c numContacts when each contact is PCWF.  This jacobian relates joint
  velocities to contact velocities and its transpose relates contact forces
  to joint torques.  

  Distances used to be in meters. Now they are in millimeters.
*/
void
Grasp::buildJacobian()
{
	double db0=0.0;
	int i,f,l,blkOffset,rowOffset;
	transf Tec;
	double *Jee,*Tv_ec,*J;
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;
  
	numDOF = hand->getNumDOF();   // +6 for palm motion

	if (externalTorques) delete [] externalTorques;
	externalTorques = new double[numDOF];
	for (i=0;i<numDOF;i++) {
		externalTorques[i] = hand->getDOF(i)->getExtForce();
	}

	if (Jacobian) delete [] Jacobian;
	Jacobian = new double[numDOF*numWrenches];
	dcopy(numDOF*numWrenches,&db0,0,Jacobian,1);
	blkOffset = 0;
  
	// J is an individual contact jacobian
	J = new double[6*numDOF];

	for (f=0;f<hand->getNumFingers();f++) {
		for (l=0;l<hand->getFinger(f)->getNumLinks();l++) {
   			contactList = hand->getFinger(f)->getLink(l)->getContacts();
			if (contactList.empty()) continue;

			Jee = getLinkJacobian(f,l);

			for (cp=contactList.begin();cp!=contactList.end();cp++) {
				if ((*cp)->getBody2() != object) continue;
				dcopy(6*numDOF,&db0,0,J,1);

				// Tec: tranform from link base to contact point
				Tec = rotate_transf(M_PI,vec3(1,0,0)) * (*cp)->getContactFrame();
				Tv_ec = new double[36];
				Tec.jacobian(Tv_ec);
				// J = Tv_ec * Jee;
				dgemm("N","N",6,numDOF,6,1.0,Tv_ec,6,Jee,6,0.0,J,6);
   				delete [] Tv_ec;

				// we only want to include the portion of J that is admissible
				// by the contact type
	    
				rowOffset = 0;
	    
				if ((*cp)->getContactDim() >= 3) {
					// copy the first row of J (dx/dtheta)
					dcopy(numDOF,J,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
      				// copy the second row of J (dy/dtheta)
					dcopy(numDOF,J+1,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
				}
   
				// copy the third row of J (dz/dtheta)
				dcopy(numDOF,J+2,6,Jacobian+blkOffset+rowOffset,numWrenches);
				rowOffset++;
    
				if ((*cp)->getContactDim() > 3) {
					// copy the sixth row of J (drz/dtheta)
					dcopy(numDOF,J+5,6,Jacobian+blkOffset+rowOffset,numWrenches);
					rowOffset++;
				}
	    		blkOffset += rowOffset;
			}
			delete [] Jee;
		}
	}  
	delete [] J;

#ifdef GRASPDEBUG
	printf("JACOBIAN:\n");
	disp_mat(stdout,Jacobian,numWrenches,numDOF,0);
#endif

}

/*!
  Tries to compute the maximum contact forces from the joint torque maxes and
  the pseudo-inverse of the transpose of the hand jacobian.  This is not
  used anymore.
*/  
void
Grasp::computeContactForceLimits()
{
  int i,dim;
  int row = numWrenches;
  int col = numDOF;
  int nrhs = 1;
  int ldb = MAX(numWrenches,numDOF);
  double *tempJac = new double [numWrenches * numDOF];
  double *B = new double[ldb * nrhs];
  double forceLimit;
  int lwork = -1;
  double *work;
  int info;

  //lwork for dgels_ 
  lwork= MAX(1,MIN(row,col)+MAX(row, MAX(col,nrhs)));
  work = new double[lwork];

  for (i=0;i<hand->getNumDOF();i++)
    B[i] = hand->getDOF(i)->getMaxForce();
  
  dcopy(numWrenches*numDOF,Jacobian,1,tempJac,1);

  dgels("T",row,col,nrhs,tempJac,row,B,ldb,work,lwork,&info);

#ifdef GRASPDEBUG
  printf("max contact forces\n");
  disp_mat(stdout,B,1,numWrenches,0);
#endif

  if (info!=0) {
    delete [] work; delete [] B; delete [] tempJac;
    printf("problem in dgels when computing the contact force limits\n");
    return;
  }

  int offset = 0;
  for (i=0;i<numContacts;i++) {
    dim = contactVec[i]->getContactDim();

    // make sure we get the normal (z) force direction
    if (dim==1)
      forceLimit = fabs(B[offset]);
    else
      forceLimit = fabs(B[offset+2]);
    
    offset+=dim;
#ifdef GRASPDEBUG
    printf("limit: %f\n",forceLimit);
#endif

    contactVec[i]->getMate()->setNormalForceLimit(forceLimit);
  }

  delete [] work;  delete [] B;  delete [] tempJac;
}

/*!
  Constructs the grasp map matrix.  This is a 6 x \c numWrenches
  (\c numWrenches = 3 * \c numContacts, when each contact is PCWF) matrix
  that computes the net object wrench given a vector of magnitudes for
  the contact wrenches.  
*/
void
Grasp::buildGraspMap()
{
  int i,curLoc = 0;

  if (graspMap) delete [] graspMap;
  graspMap = NULL;

  // count the total number of wrenches contributed by all the contacts
  numWrenches = 0;
  for(i=0;i<numContacts;i++)
    numWrenches += contactVec[i]->getContactDim();

  if (numContacts) {
    graspCounter++;
    graspMap = new double[6*numWrenches];

    /*
      This code has not been verified in a long time. Please check that the frictionEdges
      below indeed do what you expect them to
    */
    fprintf(stderr,"Unverified code! Check me first!\n");
    // add the basis wrenches of each contact to the grasp map
    for (i=0;i<numContacts;i++) {
      dcopy(6*contactVec[i]->numFrictionEdges,
	    contactVec[i]->getMate()->frictionEdges,1,graspMap+curLoc,1);
	  curLoc += 6 * contactVec[i]->getContactDim();
    }
#ifdef GRASPDEBUG
    printf("Built GraspMap (6x%d):\n",numWrenches);
    disp_mat(stdout,graspMap,6,numWrenches,1);
#endif

  }
}

/*!
  Computes the null space of the grasp map.
  Adapted from Li Han's code.
*/
void
Grasp::computeNullSpace() {

  double *tempGraspMap, *work;
  double *leftSV, *singularValues, *rightSV, *checkNull, *checkRSV;
  double db0=0.0;
  int lwork, rank, info;
  int numGElements, numCheck, i;
  int row = 6, col = numWrenches;
  
  rank=row;
  nullDim=col-rank;
  if (nullDim <= 0) return;

  if (nullSpace) free(nullSpace);

  numGElements = row*col;
  tempGraspMap=(double*) malloc(numGElements*sizeof(double));
  dcopy(numGElements,graspMap,1, tempGraspMap,1);
  
  lwork=MAX(MAX(1,3*MIN(row,col)+ MAX(row,col)),5*MIN(row,col));  //-4
  work=(double*) malloc(lwork*sizeof(double));
  singularValues=(double*)malloc(MIN(row,col)*sizeof(double));
  leftSV=(double*)malloc(row*row*sizeof(double));
  rightSV=(double*)malloc(col*col*sizeof(double));

#ifdef GRASPDEBUG  
  printf("--------- tempGraspMap ------------ \n");
  disp_mat(stdout,tempGraspMap,row,col,0);
#endif
  
  dgesvd("A","A", row, col,tempGraspMap, row, singularValues, leftSV, row,
	 rightSV, col, work, lwork, &info);
  if(info==0){

#ifdef GRASPDEBUG  
      printf("--------- singular values ------------ \n");
      disp_mat(stdout,singularValues,1,MIN(row,col),0);
      
      printf("--------- Left Singular Vectors -------- \n");
      disp_mat(stdout, leftSV, row, row,0);
      printf("--------- Right Singular Vectors -------- \n");
      disp_mat(stdout, rightSV, col, col,0);
      
      printf("-------- GraspMap*RightSV ------------ \n");
#endif

      checkRSV=(double*)malloc(col*col*sizeof(double));
      dcopy(numGElements,graspMap,1, tempGraspMap,1);
      dgemm("N","T",row,col,col,1.0,tempGraspMap,row,rightSV,col,0.0,
      checkRSV,col);
#ifdef GRASPDEBUG  
      disp_mat(stdout, checkRSV, col, col,0);
#endif
      free(checkRSV);

    nullSpace=(double *)malloc(col* nullDim *sizeof(double));
    for (i=row;i<col;i++)
      dcopy(col,rightSV+i,col,nullSpace+(i-row)*col,1);
    
#ifdef GRASPDEBUG  
      printf("---------- Null Space -----------------\n");
      disp_mat(stdout, nullSpace, col, nullDim,0);
      
      printf("---------- check null space --------- \n");
#endif
      numCheck=row*nullDim;
      checkNull=(double*)malloc(numCheck*sizeof(double));
      dcopy(numCheck, &db0, 0,checkNull,1);
      dcopy(numGElements,graspMap,1, tempGraspMap,1);
      dgemm("N","N", row, nullDim, col, 1.0, tempGraspMap, row,
      nullSpace,col, 0.0, checkNull,row);
#ifdef GRASPDEBUG  
      disp_mat(stdout, checkNull, row, nullDim, 0);
#endif
      free(checkNull);

    free(tempGraspMap);free(work); 
    free(leftSV); free(singularValues); free(rightSV);
  }
  else {
    free(tempGraspMap);free(work);
    free(leftSV); free(singularValues); free(rightSV);    
    errMsg("error in SVD of grasp map");
  }
}

/*! 
  Find the minimal norm solution to the object wrench equilibirum equation.
  Adapted from Li Han's Code - I changed the sign of the object wrench.

  The solution to the linear equation \f$ G x + F_{ext} = 0 \f$
  is \f$ x = x_0 + V z \f$
  where \f$ x_0 = G^+ * F_{ext} \f$ is the minimal normal solution.

  Grasp map:   \f$G\f$. \n
  minNormSln:  \f$x_0\f$.\n
  nullSpace:   \f$V\f$. \n
*/
void
Grasp::minimalNorm()
{
  double *tempGraspMap, *work, *incForce;
  int lwork, info;
  int numGElements, ldx0, i;
  int row = 6, col = numWrenches, nrhs = 1;

  if (minNormSln) free(minNormSln);

  incForce=(double*)malloc(row*sizeof(double));
  dcopy(row,object->getExtWrenchAcc(),1,incForce,1);

  // Grasp should generate the _negative_ object wrench
  dscal(row,-1.0,incForce,1);

#ifdef GRASPDEBUG  
  printf("---------- Object Wrench --------------------- \n");
  disp_mat(stdout, object->getExtWrenchAcc(), row, 1,0);
#endif

  minNormSln = (double*) malloc(MAX(row,col)*nrhs*sizeof(double));
  //lwork for dgels_ 
  lwork=MAX(1,MIN(row,col)+MAX(row, MAX(col,nrhs)));
  work= (double*)malloc(lwork*sizeof(double));
  
  numGElements = row*col;
  tempGraspMap=(double*) malloc(numGElements*sizeof(double));
  dcopy(numGElements,graspMap,1, tempGraspMap,1);
  
  ldx0=MAX(row,col);
  for (i=0; i<nrhs;i++)
    dcopy(row,incForce+i*row, 1, minNormSln+i*ldx0, 1);
  
  dgels("N", row, col, nrhs, tempGraspMap, row, minNormSln, ldx0, work,
	 lwork, &info);
  
  if(info==0){
#ifdef GRASPDEBUG
    printf("preset work length: %d, optimal length: %f \n", lwork, work[0]);
    printf("---------- Minimal Norm/Error Solution ---------- \n");
    disp_mat(stdout, minNormSln, nrhs, ldx0,0);
#endif
    free(work); free(tempGraspMap); free(incForce);
  }
  else {
    free(work); free(tempGraspMap); free(minNormSln); free(incForce);
    minNormSln = NULL;
    errMsg(" no exact solution exists");
  }    
}

/*!
  This sets up the contact limits lmi.  It requires the Grasp map null space,
  the minimum norm solution and the lower and upper bounds of each null space
  dimension.
    
  Adapted from Li Han's Code.
*/
double *
lmiContactLimits(int numForces, int nullDim,  double* nullSpace,
			 double * minNormSln, double* lowerBounds,
			 double* upperBounds)
{
  double *LmiLinear, *tempLower, *tempUpper;
  int row, col, nullSpaceSize;
  int i;
  
  tempLower=(double*)malloc(numForces*sizeof(double));
  dcopy(numForces, lowerBounds, 1, tempLower, 1);
  daxpy(numForces, -1.0, minNormSln, 1, tempLower, 1);  //lower-minNormSln
  dscal(numForces,-1.0,tempLower, 1);			 //-(lower-minNormSln)
  
  tempUpper=(double*)malloc(numForces*sizeof(double));	
  dcopy(numForces, upperBounds, 1, tempUpper, 1);
  daxpy(numForces, -1.0, minNormSln, 1, tempUpper, 1);  // upper - minNormSln
  
  row=2*numForces;   col=nullDim+1; 
  LmiLinear=(double*)malloc(row*col*sizeof(double));
  dcopy(numForces, tempLower, 1, LmiLinear, 1);
  dcopy(numForces, tempUpper, 1, LmiLinear+numForces, 1);
  
  for(i=0;i<nullDim; i++)
    dcopy(numForces, nullSpace+i*numForces, 1, LmiLinear+(i+1)*row,1);
  nullSpaceSize=numForces*nullDim;
  dscal(nullSpaceSize,-1.0, nullSpace, 1);
  for(i=0;i<nullDim; i++)
    dcopy(numForces, nullSpace+i*numForces, 1, LmiLinear+(i+1)*row+numForces,1);
  
  dscal(nullSpaceSize,-1.0, nullSpace, 1);
  free(tempLower); free(tempUpper);
  return(LmiLinear);
}

/*! 
  Adapted from Li Han's Code.  
*/
double *
Grasp::lmiTorqueLimits()
{
  double  db0=0.0,*tildeJ, *tildeExternalTrq;
  int i,tildeJDim=numDOF*nullDim;
  double *lmi,*lowerBounds,*upperBounds;

  /*  L=2*numWrenches;
  F_blkszs= (int *) malloc(L*sizeof(int));
  for (int i=0;i<L;i++) F_blkszs[i]=1;
  */

  L=2*numDOF;
  F_blkszs= (int *) malloc(L*sizeof(int));
  for (i=0;i<L;i++) F_blkszs[i]=1;

  lowerBounds = (double*) malloc(numDOF*sizeof(double));
  upperBounds = (double*) malloc(numDOF*sizeof(double));
  for (i=0;i<hand->getNumDOF();i++) {
    upperBounds[i] = hand->getDOF(i)->getMaxForce()*1.0e-9;
    lowerBounds[i] = - hand->getDOF(i)->getMaxForce()*1.0e-9;
  }

  tildeJ=(double*) malloc(tildeJDim*sizeof(double));
  dcopy(tildeJDim, &db0, 0, tildeJ,1);

  dgemm("T","N", numDOF, nullDim, numWrenches, 1.0, Jacobian, numWrenches,
	 nullSpace, numWrenches, 0.0, tildeJ, numDOF);

  tildeExternalTrq=(double*) malloc(numDOF*sizeof(double));
  dcopy(numDOF, externalTorques, 1, tildeExternalTrq, 1);
  
  dgemv("T", numWrenches, numDOF, 1.0, Jacobian, numWrenches, minNormSln, 1,
	 1.0, tildeExternalTrq, 1);

#ifdef GRASPDEBUG  
  printf("tildeExternalTrq:\n");
  disp_mat(stdout,tildeExternalTrq,1,numDOF,0);
#endif

  lmi = lmiContactLimits(numDOF, nullDim, tildeJ, tildeExternalTrq,
		       lowerBounds, upperBounds);

  free(tildeJ); free(tildeExternalTrq);  free(lowerBounds); free(upperBounds);
  return(lmi);
}

void
Grasp::lmiFL(double *lmi,int rowInit, int colInit, int totalRow)
{
  int x3_post=colInit*totalRow+rowInit;
  lmi[x3_post]=1.0;
}

void
Grasp::lmiPCWF(double cof, double *lmi,int rowInit, int colInit,
		    int totalRow)
{
  int x1_post, x2_post, x3_post;

  x1_post=colInit*totalRow+rowInit+2;
  lmi[x1_post]=1.0;
  
  x2_post=(colInit+1)*totalRow+rowInit+4;
  lmi[x2_post]=1.0;
  
  x3_post=(colInit+2)*totalRow+rowInit;
  lmi[x3_post]=cof;
  lmi[x3_post+3]=cof;
  lmi[x3_post+5]=cof;
}

void
Grasp::lmiSFCE(double cof, double cof_t,double *lmi,int rowInit,
		    int colInit, int totalRow)
{
  int x1_post, x2_post, x3_post, x4_post;
  double alpha, beta;
  
  alpha=1.0/sqrt(cof);
  beta =1.0/sqrt(cof_t);
  
  x1_post=colInit*totalRow+rowInit+3;
  lmi[x1_post]=alpha;
  
  x2_post=(colInit+1)*totalRow+rowInit+6;
  lmi[x2_post]=alpha;
  
  x3_post=(colInit+2)*totalRow+rowInit;
  lmi[x3_post]=1.0;
  lmi[x3_post+4]=1.0;
  lmi[x3_post+7]=1.0;
  lmi[x3_post+9]=1.0;

  x4_post=(colInit+3)*totalRow+rowInit+8;
  lmi[x4_post]=beta;
}

void
Grasp::lmiSFCL(double cof, double cof_t,double *lmi,int rowInit,
		    int colInit, int totalRow)
{
  int colFirst;
  double ratio = cof/cof_t;

  // The 1 dimensional index for the first item of this LMI block
  // in the x_i column

  //x_1
  colFirst=colInit* totalRow+rowInit;
  lmi[colFirst+9]=1.0;
  lmi[colFirst+24]=1.0;
  
  //x_2
  colFirst=(colInit+1)*totalRow+rowInit;
  lmi[colFirst+14]=1.0;
  lmi[colFirst+26]=1.0;
    
  //x_3
  colFirst=(colInit+2)*totalRow+rowInit;
  lmi[colFirst]   =1.0;
  lmi[colFirst+7] =cof;
  lmi[colFirst+13]=cof;
  lmi[colFirst+18]=cof;
  lmi[colFirst+22]=cof;
  lmi[colFirst+25]=cof;
  lmi[colFirst+27]=cof;
  
  //x_4
  colFirst=(colInit+3)*totalRow+rowInit;
  lmi[colFirst+7] =ratio;
  lmi[colFirst+13]=ratio;
  lmi[colFirst+18]=ratio;
  lmi[colFirst+22]=-ratio;
  lmi[colFirst+25]=-ratio;
  lmi[colFirst+27]=-ratio;
}

// note that LMI is stored in a packed form where the whole lower triangle of
// each P_i is stored as a single column (remember: column-major format)
//
double *
Grasp::lmiFrictionCones()
{
  double db0=0.0,*initLMI, *finalLMI;
  int lmiDim;
  int row=0, col=0, *blockRowIndex, *blockColIndex;
  int i, initSize, finalSize;

  blockRowIndex = new int[numContacts];
  blockColIndex = new int[numContacts];

  K=numContacts;
  G_blkszs=(int *)malloc(K*sizeof(int));

  for (i=0;i<numContacts;i++) {
    blockRowIndex[i]=row;
    blockColIndex[i]=col;
    lmiDim = contactVec[i]->getLmiDim();
    row += lmiDim*(lmiDim+1)/2;  // size of the lower triangle
    col += contactVec[i]->getContactDim();
    G_blkszs[i]=lmiDim;
  }

  initSize=row*col;
  initLMI=(double*)malloc(initSize*sizeof(double));
  dcopy(initSize,&db0, 0, initLMI, 1);

  for (i=0;i<numContacts;i++) {
    switch(contactVec[i]->getFrictionType()) {
      case FL:   lmiFL(initLMI,blockRowIndex[i],blockColIndex[i], row);
	         break;
      case PCWF: lmiPCWF(contactVec[i]->getCof(),initLMI,blockRowIndex[i],
			 blockColIndex[i], row);
                 break;
      case SFCE: assert(0); //not handled
			 //lmiSFCE(contactVec[i]->getCof(),
			 //contactVec[i]->getTorsionalCof(),initLMI,
			 //blockRowIndex[i], blockColIndex[i], row);
                 break;
      case SFCL: assert(0); //not handled
		     //lmiSFCL(contactVec[i]->getCof(),
			 //contactVec[i]->getTorsionalCof(),initLMI,
			 //blockRowIndex[i], blockColIndex[i], row);
    }
  }
#ifdef GRASPDEBUG  
  printf("------------- Init LMI(%-dx%-d) ----------------------\n",row, col);
  disp_mat(stdout, initLMI, row, col,0);
#endif

  finalSize=row*(nullDim+1);
  
  finalLMI = (double*) malloc(finalSize*sizeof(double));
  dcopy(finalSize,&db0, 0, finalLMI, 1);
  dgemm("N","N",row, nullDim, col, 1.0, initLMI, row, nullSpace, col, 
	 0.0,finalLMI+row, row);
  dgemv("N",row,col, 1.0, initLMI, row, minNormSln, 1, 0.0, finalLMI, 1);
  free(initLMI);
  delete [] blockRowIndex;
  delete [] blockColIndex;
  GPkRow = row;
  return(finalLMI);
}

double *
Grasp::weightVec()
{
  double  db0=0.0,*initWeights, *finalWeights;
  int i,dim,blockIndex=0;
  
  initWeights=(double*)malloc(numWrenches*sizeof(double));
  for (i=0;i<numContacts;i++) {
    dim = contactVec[i]->getContactDim();
    dcopy(dim,&db0,1,initWeights+blockIndex,1);

    // temporary preset weights
    if (dim==1)
      *(initWeights+blockIndex) = Grasp::GFO_WEIGHT_FACTOR;
    else
      *(initWeights+blockIndex+2) = Grasp::GFO_WEIGHT_FACTOR;
    
    blockIndex += dim;
  }

#ifdef GRASPDEBUG
  printf("------------ Init Weight Vector(%d) -------------\n",numWrenches);
  disp_mat(stdout, initWeights,1,numWrenches,0);
  
      printf("---------- Null Space -----------------\n");
      disp_mat(stdout, nullSpace, numWrenches, nullDim,0);
#endif
      
  finalWeights=(double*)malloc(nullDim*sizeof(double));
  dcopy(nullDim,&db0,0,finalWeights,1);
  dgemv("T",numWrenches,nullDim,1.0,nullSpace,numWrenches,initWeights,1,
	0.0,finalWeights, 1);
  
  constOffset = ddot(numWrenches,initWeights,1,minNormSln,1);

#ifdef GRASPDEBUG
  printf("------------ Final Weight Vector(%d) for new variables z & %lf -------------\n",nullDim, constOffset);
  disp_mat(stdout, finalWeights,1,nullDim,0);
#endif

  free(initWeights);
  return(finalWeights);
}

/*!
  Combines LMIs \a lmi1 >0 and \a lmi2 >0 into the LMI \n
  [lmi1    0]\n
  [0    lmi2] > 0
*/
double *
combineLMIs(int numVariables, double* lmi1, int lmi1RowSz, double * lmi2,
	    int lmi2RowSz) {

  double * combinedLMI;
  int numElements, totalRowSz;
  int i;
  
  totalRowSz=lmi1RowSz+lmi2RowSz;
  numElements=(numVariables+1)*totalRowSz;
  combinedLMI=(double*)malloc(numElements*sizeof(double));
  for(i=0;i<=numVariables;i++){
    dcopy(lmi1RowSz,lmi1+i*lmi1RowSz,1,combinedLMI+i*totalRowSz,1);
    dcopy(lmi2RowSz,lmi2+i*lmi2RowSz,1,combinedLMI+i*totalRowSz+lmi1RowSz,1);
  }
  return(combinedLMI);
}

/*!
  Prepare the parameters used by maxdet package and
  use it to solve the maxdet optimization problem.
*/
double * maxdet_wrap(int m, int L, double *F, int *F_blkszs,
                   int K, double *G, int *G_blkszs,
                   double *c, double *z0, double *Z, double *W,
                   double gamma, double abstol, double reltol,
                   int * pNTiters, int negativeFlag, FILE* pRstFile)


{
  register int i;
  int    n, l, max_n, max_l, F_sz, G_sz;  
  int    ptr_size;
  
  int *iwork, lwork;
  double db0=0.0,ul[2], *hist, *truncatedHist, *work;
  int m3, m2, sourceCol, destCol, destSize;
  int info, info2;
  
  
  //  struct tms before,after;
  
  // Gets rid of compiler warning
#ifndef GRASPDBG
  pRstFile = NULL;
#endif
  
  for (i=0; i<L; i++) {
    if (F_blkszs[i] <= 0) {
      pr_error("Elements of F_blkszs must be positive.\n");
      errorOccurred = true;
      return NULL;
    }
  }
  for (i=0; i<K; i++) {
    if (G_blkszs[i] <= 0) {
      pr_error("Elements of G_blkszs must be positive.\n");
      errorOccurred = true;
      return NULL;
    }
  }
  
  /* various dimensions
   * n, l: dimensions
   * F_sz, G_sz: length in packed storage
   * F_upsz, G_upsz: length in unpacked storage
   * max_n, max_l: max block size */
  for (i=0, n=0, F_sz=0, max_n=0; i<L; i++) {
    n += F_blkszs[i];
    F_sz += (F_blkszs[i]*(F_blkszs[i]+1))/2;
    max_n = MAX(max_n, F_blkszs[i]);
  }
  for (i=0, l=0, G_sz=0, max_l=0; i<K; i++) {
    l += G_blkszs[i];
    G_sz += (G_blkszs[i]*(G_blkszs[i]+1))/2;
    max_l = MAX(max_l, G_blkszs[i]);
  }
  
  /* hist (5th output argument) */
  ptr_size =(3+m)*(*pNTiters);
  //    ptr_size =(3+m)*(NTiters);
  hist=(double *) malloc(ptr_size*sizeof(double));
  dcopy(ptr_size,&db0, 0, hist, 1);
  
  /* allocate work space */
  lwork = (2*m+5)*(F_sz+G_sz) + 2*(n+l) +
    MAX(m+(F_sz+G_sz)*NB,MAX(3*(m+SQR(m)+MAX(G_sz,F_sz)),
			     MAX(3*(MAX(max_l,max_n)+MAX(G_sz,F_sz)),
				 MAX(G_sz+3*max_l,F_sz+3*max_n))));
  work = (double *) malloc(lwork*sizeof(double));
  dcopy(lwork,&db0,0,work,1);
  iwork = (int *) malloc(10*m*sizeof(int));
  for(i=0;i<10*m;i++)	iwork[i]=0;
  
  //  times(&before);
  info2=maxdet(m,L, (double*)F,(int *) F_blkszs,K,(double *) G,
	       (int *) G_blkszs,(double *)c,(double *)z0,(double *)Z,
	       (double *)W, (double *)ul, (double*)hist,gamma,abstol,reltol,
	       pNTiters,work,lwork,iwork,&info,negativeFlag);
  
  if (info2) {
      free(work); free(iwork); free(hist);
      errorOccurred = true;
      pr_error("Error in maxdet.\n");
      return NULL;
  }
  
  // times(&after);
  
  //#ifndef REAL_RUN
  //  fprintf(pRstFile,"times:  User time: %f seconds\n",(float) (after.tms_utime-before.tms_utime)/sysconf(CLK_TCK));
  //#endif
  
  // infostr 
#ifdef GRASPDEBUG
  switch (info) {
  case 1:
    fprintf(pRstFile, "  maximum Newton iteration exceeded\n");
    break;
  case 2:
    fprintf(pRstFile, "  absolute tolerance reached\n");
    break;
  case 3:
    fprintf(pRstFile, "  relative tolerance reached\n");
    break;
  case 4:
    fprintf(pRstFile, "  negative objective value reached\n");
    break;
  default:
    printf("error occurred in maxdet\n"); 
    exit(-1);
  }  
#endif

  //  truncate hist
  destCol=0;
  m3=m+3; m2=m3 -1;
  for(sourceCol=0;sourceCol<*pNTiters;sourceCol++) 
    if(hist[sourceCol*m3+m2]!=0.0){
      dcopy(m3,hist+sourceCol*m3,1,hist+destCol*m3,1);
      destCol++;
    }
  //        printf("init Col: %d  final Col: %d \n", sourceCol, destCol);
  destSize=destCol*m3;
  truncatedHist=(double *)malloc(destSize*sizeof(double));
  dcopy(destSize, hist, 1, truncatedHist, 1);
  *pNTiters=destCol;
  
  
  /* free matrices allocated */
  free(work); free(iwork); free(hist);
  
  return(truncatedHist);
}

/*!
  Solves the question of whether there is a feasible set of contact forces
  that will result in equilibrium for the object.
*/
void
Grasp::feasibilityAnalysis()
{
  double *eigF, minEigF;
  double *eigG, minEigG;
  double *work, *hist;
  
  double *identityF, *identityG;
  double *expandF, *expandG;
  double *newF, *newG;
  int    *newF_blkszs,  newG_blkszs;
  double db0=0.0,t0, *x0,*Z0, W0, *c;
  int    newM, newL, newK, newPkSz,initSize;
  
  int F_pksz=0, G_pksz=0;
  int F_dim=0, G_dim=0;
  int blkSize, maxBlockSize=0;
  int i,m,workSize;
  //int  negativeFlag=TRUE;
    
  eigF = new double[numDOF*2]; // max dimension of torque limit LMI
  eigG = new double[numContacts*7];    // max dimension of friction LMI
  
  m = nullDim;
  newM=m+1;
  newG=(double*)malloc((newM+1)*sizeof(double));
  newG[0]=1.0;
  dcopy(newM,&db0,0,newG+1,1);
  newK=1;
  newG_blkszs=1;
  W0=0.0;
    
  c=(double*)malloc(newM*sizeof(double));
  dcopy(m,&db0,0,c,1);
  c[m]=1.0;
  
  for(i=0;i<L;i++){
    blkSize=F_blkszs[i];
    F_dim+=blkSize;
    F_pksz+=(blkSize+1)*blkSize/2;
    maxBlockSize=MAX(maxBlockSize,blkSize);
  }
  workSize=F_pksz+3*maxBlockSize;
  work=(double*)malloc(workSize*sizeof(double));
  minEigF=eig_val(eigF,F,L,F_blkszs,F_pksz,work);
  free(work);
  
  maxBlockSize=0;
  for(i=0;i<K;i++){
    blkSize=G_blkszs[i];
    G_dim+=blkSize;
    G_pksz+=(blkSize+1)*blkSize/2;
    maxBlockSize=MAX(maxBlockSize,blkSize);
  }
  workSize=G_pksz+3*maxBlockSize;
  work=(double*)malloc(workSize*sizeof(double));
  minEigG=eig_val(eigG,G,K,G_blkszs,G_pksz,work);
  free(work);
  
#ifdef GRASPDEBUG
  fprintf(stderr,"minEigF: %f  minEigG:%f \n",minEigF, minEigG);
  fprintf(stderr," ------- EigF(%d) ------ \n",F_dim);
  disp_mat(stderr,eigF,1,F_dim,0);
  fprintf(stderr," ------- EigG(%d) ------ \n",G_dim);
  disp_mat(stderr,eigG,1,G_dim,0);
#endif
  
  t0=MAX(-1.1*MIN(minEigF,minEigG),1.0);
  x0=(double*)malloc(newM*sizeof(double));
  dcopy(m,&db0,0,x0,1);
  x0[m]=t0;
  
  identityF=(double*)block_Identity(L,F_blkszs,F_pksz);
  identityG=(double*)block_Identity(K,G_blkszs,G_pksz);

#ifdef GRASPDEBUG
  fprintf(stderr,"------------- Identity_F ---------------- \n");
  disp_mat(stderr,identityF,1,F_pksz,0);
  fprintf(stderr,"------------- Identity_G ---------------- \n");
  disp_mat(stderr,identityG,1,G_pksz,0);
#endif
  
  expandF=(double*)malloc((newM+1)*F_pksz*sizeof(double));
  initSize=(m+1)*F_pksz;
  dcopy(initSize,F,1,expandF,1);
  dcopy(F_pksz,identityF,1,expandF+initSize,1);
  free(identityF);
  
  expandG=(double*)malloc((newM+1)*G_pksz*sizeof(double));
  initSize=(m+1)*G_pksz;
  dcopy(initSize,G,1,expandG,1);
  dcopy(G_pksz,identityG,1,expandG+initSize,1);
  free(identityG);
  
  newF=(double*) combineLMIs(newM,expandF,F_pksz,expandG,G_pksz);
  free(expandF); free(expandG);
  
  newL=L+K;
  newF_blkszs=(int*)malloc(newL*sizeof(int));
  for(i=0;i<L;i++)  newF_blkszs[i]=F_blkszs[i];
  for(i=0;i<K;i++)  newF_blkszs[L+i]=G_blkszs[i];
  
  newPkSz=F_pksz+G_pksz;
  Z0=(double*)malloc(newPkSz*sizeof(double));
  dcopy(newPkSz,&db0,0,Z0,1);
  
#ifdef GRASPDEBUG
  fprintf(stderr,"------------- Combined LMI (%d x %d) ---------------- \n", F_pksz+G_pksz, newM+1);
  disp_mat(stderr,newF,F_pksz+G_pksz, newM+1,0);
  fprintf(stderr,"------------- Combined LMI Block Sizes(%d) ---------------- \n", newL);
  disp_imat(stderr,newF_blkszs,1, newL,0);
#endif
  
  fprintf(stderr,"Checking grasping force feasibility...\n");
  hist = maxdet_wrap(newM, newL, newF, newF_blkszs, newK, newG, &newG_blkszs,
		     c, x0,Z0, &W0, gamma, abstol,  reltol, &feasNTiters,
		     negativeFlag,pRstFile);

  if (errorOccurred) {
    free(newF); free(newF_blkszs); free(Z0);
    free(newG); free(c); free(x0);
    delete [] eigF; delete [] eigG;
    return;
  }

  if (initz0) free(initz0);
  initz0=(double *)malloc(nullDim*sizeof(double));
  dcopy(m,x0,1,initz0,1);

  if (feasZHistory) free(feasZHistory);
  feasZHistory=(double*)malloc((m+3)*(feasNTiters)*sizeof(double));
  for(i=0;i<feasNTiters;i++){
    dcopy(newM,hist+i*(newM+3),1,feasZHistory+i*(m+3),1);
    dcopy(2,hist+i*(newM+3)+newM+1, 1, feasZHistory+i*(m+3)+m+1,1);
  }
  if(x0[m]<0) {
    feasible=TRUE;
#ifdef GRASPDEBUG
    printf("\n Feasible! min eig value: %f \n",-x0[m]);
#endif
  }
  else feasible=FALSE;
  
  free(newF); free(newF_blkszs); free(Z0);
  free(newG); free(c); free(x0); free(hist);
  delete [] eigF; delete [] eigG;
}

/*!
  Formulate and solve the optimization 4 problem (See the LMI paper)
*/
void
Grasp::optm_EffortBarrier()
{
  int    newL, newK;
  double *newF, *newG;
  int    newF_blkszs,  *newG_blkszs;
  double db0=0.0,Z, *W;
  int F_pksz=0, G_pksz=0, newPkSz, i, blkSize;
  int m;
  
  
  m=nullDim;
  if (optmz0) free(optmz0);
  optmz0=(double*)malloc(m*sizeof(double));
  dcopy(m, initz0, 1, optmz0, 1);

  newL=1;
  newF_blkszs=1;
  newF=(double*)malloc((m+1)*sizeof(double));
  newF[0]=1.0;
  dcopy(m,&db0,0,newF+1,1);
  Z=db0;
  
  newK=L+K;
  newG_blkszs=(int*)malloc(newK*sizeof(int));
  for(i=0;i<L;i++){
    blkSize=F_blkszs[i];
    newG_blkszs[i]= blkSize;
    F_pksz+=(blkSize+1)*blkSize/2;
  }
  for(i=0;i<K;i++){
    blkSize=G_blkszs[i];
    newG_blkszs[L+i]= blkSize;
    G_pksz+=(blkSize+1)*blkSize/2;
  }
  newG = (double*) combineLMIs(m,F,F_pksz,G,G_pksz);
  newPkSz=F_pksz+G_pksz;
  W=(double*)malloc(newPkSz*sizeof(double));
  dcopy(newPkSz,&db0,0,W,1);

  if (optmZHistory) free(optmZHistory);
  fprintf(stderr,"Optimizing...\n");
  optmZHistory =  maxdet_wrap(m,newL, newF, &newF_blkszs, newK, newG,
			      newG_blkszs, c, optmz0, &Z, W, gamma, abstol,
			      reltol, &optmNTiters, FALSE, pRstFile);

  if (errorOccurred) {
    free(newF); free(newG_blkszs); free(W);
    free(newG);
    return;
  }
  
  // transform the optimal z value to the optimal x value 
  if (optmx0) free(optmx0);
  optmx0=(double*)malloc(numWrenches*sizeof(double));
  dgemv("N", numWrenches, nullDim, 1.0, nullSpace, numWrenches, 
	optmz0,1, 0.0, optmx0,1);
  daxpy(numWrenches, 1.0, minNormSln, 1, optmx0, 1);

  free(newF); 
  free(newG); free(newG_blkszs); free(W);
}

/*!
  Compute the value of the objective function for each set of z values
  stores the optimal z value and the corresponding objective value
  in \c extendOptmz0.
*/
void
Grasp::computeObjectives()
{
  double *tempG, *zHistory,objective;
  int    zHistRowDim;
  
  double db0=0.0,minEigG, *eigG, *work;
  int workSize, maxBlockSize=0, lmiDim=0;
  int i,j;

  /* put the initial z value in the first row of the history, then
     copy z history after that */
  zHistRowDim=nullDim+3;
  if (extendOptmZHistory) free(extendOptmZHistory);
  extendOptmZHistory=(double*) malloc(zHistRowDim*(optmNTiters+1)*
					   sizeof(double));
  dcopy(zHistRowDim,initz0,1,extendOptmZHistory,1);
  dcopy(zHistRowDim*optmNTiters,optmZHistory,1,extendOptmZHistory+
	zHistRowDim,1);
  

  zHistory = extendOptmZHistory;

  tempG=(double *)malloc(GPkRow*sizeof(double));
  
  for(i=0;i<K;i++){
    maxBlockSize=MAX(maxBlockSize, G_blkszs[i]);
    lmiDim+=G_blkszs[i];
  }
  workSize=GPkRow+3*maxBlockSize;
  work=(double *)malloc(workSize*sizeof(double));
  eigG=(double *)malloc(lmiDim*sizeof(double));


  for(i=0;i<optmNTiters+1;i++) {
    objective=ddot(nullDim,c,1,zHistory+i*zHistRowDim,1);
    objective+=constOffset;
    
    dcopy(GPkRow,G,1, tempG,1);
    dgemv("N", GPkRow, nullDim, 1.0, G+GPkRow, GPkRow,
	   zHistory+i*zHistRowDim,1,1.0, tempG, 1);
    
    dcopy(workSize,&db0,0,work,1);
    dcopy(lmiDim,&db0,0,eigG,1);
    minEigG=eig_val(eigG,tempG,K,G_blkszs,GPkRow,work);
    if(minEigG<0)	errMsg("G(x) not positive definite");
    for(j=0;j<lmiDim; j++)
      objective -=log(eigG[j]);
    
    zHistory[i*zHistRowDim+nullDim]=objective;
  }
  free(tempG);  free(work); free(eigG);
}


/*!
  Transform z values to values of grasp force x.
*/
double *
Grasp::xzHistoryTransfrom(double *zHistory,int numIters)
{
  double db0=0.0,*xHistory;
  int xHistSize, xHistRowSize, zHistRowSize, i;


  xHistRowSize=numWrenches+1; 		// X+objective
  xHistSize=xHistRowSize*numIters;
  xHistory=(double*)malloc(xHistSize*sizeof(double));
  dcopy(xHistSize,&db0,0,xHistory, 1);
  
  zHistRowSize=nullDim+3;

  for(i=0;i<numIters;i++){
    dgemv("N", numWrenches, nullDim, 1.0, nullSpace, numWrenches, 
	   zHistory+i*zHistRowSize,1, 0.0, xHistory+i*xHistRowSize,1);
    daxpy(numWrenches, 1.0, minNormSln, 1, xHistory+i*xHistRowSize, 1);
  }
  dcopy(numIters,zHistory+nullDim, zHistRowSize, xHistory+numWrenches,
	xHistRowSize);

  return(xHistory);
}

/*!
  Main GFO routine.
  Adapted from Li Han's code.
*/
int
Grasp::findOptimalGraspForce()
{
  int i;

  feasible = FALSE;

  if (numWrenches < 6) {
    printf("More contacts are necessary before solving for the optimal grasp force\n");
    return FAILURE;
  }

#ifdef GRASPDEBUG  
  char rstFile[40];
  sprintf(rstFile,"Grasp%02d.rst",graspCounter);
  if ((pRstFile=fopen(rstFile,"w"))==NULL)
    FatalErrMsg("failed to open result file");

  //  graspMap = ReadGraspMap("mytest.G",18);
  printf("---------------- Solve Grasping Force Equation ---------------- \n");
#endif

  minimalNorm();
  // if (optmx0) {free(optmx0); optmx0=NULL;}
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile);return FAILURE;}

  computeNullSpace();
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile);return FAILURE;}

  if (nullDim == 0) {
    printf("The dimension of the null space is 0. Cannot solve for the optimal grasp force.\n");

    return FAILURE;
  }
 
#ifdef GRASPDEBUG  
  printf("----------- Prepare LMIs for Torque/Contact Limits --------------- \n");
#endif

  if (F) free(F);
  F = lmiTorqueLimits();
 
#ifdef GRASPDEBUG
  printf("----------- Prepare LMIs for Friction Cones --------------- \n");
#endif
  
  if (G) free(G);
  G = lmiFrictionCones();

#ifdef GRASPDEBUG
  printf("--------- Prepare Weight Vectors in the Objective Function ------- \n");
#endif

  if (c) free(c);
  c = weightVec();
  
#ifdef GRAPSDEBUG
  printf("\n -------- The  Feasibility Phase ----------------------- \n");
  fprintf(pRstFile, "\n -------- The  Feasibility Phase ----------------------- \n");
#endif

  feasNTiters=MAX_FEAS_LOOPS;  
  feasibilityAnalysis();
  if (errorOccurred) {errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}

  if (feasXHistory) free(feasXHistory);
  feasXHistory=xzHistoryTransfrom(feasZHistory, feasNTiters);

  if(feasible) {
#ifdef GRASPDEBUG
    printf("\n -------- The  Optimization  Phase ----------------------- \n");
    fprintf(pRstFile, "\n -------- The  Optimization  Phase ----------------------- \n");
#endif

    optmNTiters=MAX_OPTM_LOOPS;
    optm_EffortBarrier();
    if (errorOccurred) {feasible = FALSE; errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}
    
    computeObjectives();
    if (errorOccurred) {feasible = FALSE; errorOccurred = false; if (pRstFile) fclose(pRstFile); return FAILURE;}

    if (optmXHistory) free(optmXHistory);
    optmXHistory = xzHistoryTransfrom(extendOptmZHistory, optmNTiters+1);
    
    printf("OPTIMAL CONTACT FORCES:\n");
    disp_mat(stdout,optmx0,1,numWrenches,0);

    //    double *testOut = new double[6];
    //    dgemv("N",6,numWrenches,1.0,graspMap,6,optmx0,1,0.0,testOut,1);
    //    printf("CHECK:\n");
    //    for (i=0;i<6;i++)
    //      printf("%15.12le ",testOut[i]);
    //    printf("\n");
    //    delete [] testOut;

    if (optTorques) delete [] optTorques;
    optTorques = new double[numDOF];

    dcopy(numDOF,externalTorques,1,optTorques,1);
    dgemv("T", numWrenches, numDOF, 1.0, Jacobian, numWrenches,
	  optmx0, 1,1.0, optTorques, 1);

    printf("OPTIMAL TORQUES:\n");
    disp_mat(stdout,optTorques,1,numDOF,0);

    int offset = 0;
    for (i=0;i<numContacts;i++) {
      contactVec[i]->getMate()->setContactForce(optmx0+offset);
      offset += contactVec[i]->getContactDim();
    }
  }
  else {
    // prompt that the current grasp force is infeasible.
    printf("not feasible.\n");
#ifdef GRASPDEBUG
    fprintf(pRstFile,"\n --------  Problem Infeasible ----------------------- \n");
#endif
    return FAILURE;
  }


// save the problem parameters, the timing results and the feaibility as well as 
// optimization results at the last simulation step to file argv[1].rst
#ifdef GRASPDEBUG
  fprintf(pRstFile, "-------------- Jacobian (%-dx%-d) -------------- \n",numWrenches,numDOF);
  disp_mat(pRstFile, Jacobian, numWrenches, numDOF, 0);

  fprintf(pRstFile, "-------------- External Torques (%-d)------------------ \n",numDOF);
  disp_mat(pRstFile, externalTorques,1,numDOF,0);
  
  fprintf(pRstFile, "-------------- Grasp Map (6x%-d) -------------- \n", numWrenches);
  disp_mat(pRstFile, graspMap,6, numWrenches,0);
  
  fprintf(pRstFile, "-------------- Object Wrench (6) -------------- \n");
  disp_mat(pRstFile, object->getExtWrenchAcc(), 1, 6, 0);
  
  fprintf(pRstFile, "-------------- Minimal Norm Solution (%-d)------------------ \n",numWrenches);
  disp_mat(pRstFile, minNormSln,1,numWrenches,0);
  
  fprintf(pRstFile, "-------------- Admissible Null Space (%-dx%-d)------------------ \n",numWrenches, nullDim);
  disp_mat(pRstFile, nullSpace, numWrenches, nullDim,0);
  
  fprintf(pRstFile, "-------------- F blkszs (%d)------------- \n",L);
  disp_imat(pRstFile, F_blkszs, 1, L, 0);
  
  fprintf(pRstFile, "-------------- F (%-dx%-d) ------------- \n",L,nullDim+1);
  disp_mat(pRstFile, F, L, nullDim+1,0);
  
  fprintf(pRstFile, "-------------- G blkszs (%d)------------- \n",K);
  disp_imat(pRstFile, G_blkszs, 1, K,0);
  
  fprintf(pRstFile,"-------------- G (%-dx%-d) -------------- \n",GPkRow,
	  nullDim+1);
  disp_mat(pRstFile, G, GPkRow, nullDim+1,0);
  
  fprintf(pRstFile,"-------- Final Weight Vector(%d) and Offset: %f -------\n",
	  nullDim,constOffset);
  disp_mat(pRstFile, c,1,nullDim,0);
  
  fprintf(pRstFile,"------------- Initial Feasible z and History(%dx%d) -------------- \n",
	  3+nullDim,feasNTiters);
  disp_mat(pRstFile, initz0,1,nullDim,0);
  disp_mat(pRstFile, feasZHistory, 3+nullDim, feasNTiters, 0);
  
  fprintf(pRstFile,"-------------  Feasible X History(%dx%d) ---------------- \n",
	  numWrenches+1,feasNTiters);
  disp_mat(pRstFile, feasXHistory, numWrenches+1, feasNTiters, 0);
  
  if(feasible) {
    fprintf(pRstFile,"------------- Optimal z and History(%dx%d)------------------ \n",
	    3+nullDim,optmNTiters+1);
    disp_mat(pRstFile, optmz0,1,nullDim,0);
    disp_mat(pRstFile, extendOptmZHistory, 3+nullDim, optmNTiters+1, 0);
    
    fprintf(pRstFile,"------------- Optimal X and History(%dx%d)------------------ \n",
	    numWrenches+1,optmNTiters+1);
    disp_mat(pRstFile, optmXHistory, numWrenches+1, optmNTiters+1, 0);
  }

  fclose(pRstFile);  
#endif

  return SUCCESS;

}

/*! Computes the contact jacobian J. J relates joint angle motion to 
	contact motion. Its transpose, JT, relates contact forces to joint
	forces. 

	If \a worldCoords is true, all the computations are performed in 
	world coordinates, thus J will be used with joint angle and contact
	force vectors alse expressed in world coordinates. If not, J is in
	local coordinate system for each link.
*/
Matrix 
Grasp::contactJacobian(bool worldCoords)
{
	//compute individual chain jacobians
	int numRows = 0, numCols = 0;
	std::list<Matrix*> chainJacobians;
	for (int c=0; c<hand->getNumChains(); c++) {
		//start with the full jacobian (6dof per dynamic joint)
		Matrix fullChainJac( hand->getChain(c)->activeContactJacobian(worldCoords) );
		//and keep only actuated dof's
		Matrix* actuatedChainJac = new Matrix(hand->getChain(c)->actuatedJacobian(fullChainJac));
		chainJacobians.push_back( actuatedChainJac );
		numRows += chainJacobians.back()->rows();
		numCols += chainJacobians.back()->cols();
	}
	//allocate grasp jacobian matrix and init with zeroes
	Matrix J(Matrix::ZEROES(numRows, numCols));
	//copy individual chain jacobians on diagonal
	numRows = numCols = 0;
	for (int c=0; c<hand->getNumChains(); c++) {
		J.copySubMatrix(numRows, numCols, *chainJacobians.front());
		numRows += chainJacobians.front()->rows();
		numCols += chainJacobians.front()->cols();
		delete chainJacobians.front();
		chainJacobians.pop_front();
	}
	return J;
}

/*! Performs quasistatic contact force analysis.*/
int 
Grasp::computeQuasistaticForces(bool useDynamicDofForce)
{
	//todo: for now this assumes all link contacts are also on the grasped object

	//get joint torques for the entire robot
	Matrix robotTau(hand->staticJointTorques(useDynamicDofForce));

	//iros09: if the hand is touching anything other then the object, abort
	for (int c=0; c<hand->getNumChains(); c++) {
		if ( hand->getChain(c)->getNumContacts(NULL) !=
			hand->getChain(c)->getNumContacts(object) ) {
				return 1;
			}
	}

	bool freeChainForces = false;
	int numContacts = 0;
	//matrices J, D, F and R are assembled as block-diagonal from their chain equivalents
	//matrix tau as a column vector from chain equivalents
	std::list<Matrix*> chainJ, chainD, chainF, chainR, chainTau;
	for(int c=0; c<hand->getNumChains(); c++) {
		//for now, we look at all contacts
		numContacts += hand->getChain(c)->getNumContacts(NULL);
		Matrix fullChainJac( hand->getChain(c)->activeContactJacobian(false) );
		chainJ.push_back(new Matrix( hand->getChain(c)->actuatedJacobian(fullChainJac) ) );
		Matrix *cTau = new Matrix( hand->getChain(c)->jointTorquesVector(robotTau) );
		if (chainJ.back()->rows()) {
			//the chain has some contacts; push back its joint torques
			chainTau.push_back(cTau);
		} else {
			//the chain has no contacts
			if (cTau->absMax() > 2.0) {
				//but the chain has some forces! these can not be balanced
				//as the chain has no contacts!
				freeChainForces = true;
				//DBGA("Joint torque " << cTau->absMax() << " on chain " << c << " with no contact");
			}
			//push back an empty matrix
			delete cTau; chainTau.push_back(new Matrix(0,0));
		}
		chainD.push_back(new Matrix( hand->getChain(c)->frictionForcesMatrix() ) );
		chainF.push_back(new Matrix( hand->getChain(c)->frictionConstraintsMatrix() ) );
		chainR.push_back(new Matrix( hand->getChain(c)->contactForcesToWorldWrenches() ) );
	}
	Matrix J( Matrix::BLOCKDIAG(&chainJ) );
	Matrix D( Matrix::BLOCKDIAG(&chainD) );
	Matrix F( Matrix::BLOCKDIAG(&chainF) );
	Matrix R( Matrix::BLOCKDIAG(&chainR) );
	Matrix tau( Matrix::BLOCKCOLUMN(&chainTau) );
	for(int c=0; c<hand->getNumChains(); c++) {
		delete chainJ.front(); chainJ.pop_front();
		delete chainD.front(); chainD.pop_front();
		delete chainF.front(); chainF.pop_front();
		delete chainR.front(); chainR.pop_front();
		delete chainTau.front(); chainTau.pop_front();
	}
	//if there are no contacts, nothing to compute!
	if (!J.rows() || !J.cols()) return 0;
	//if there are forces on chains with no contacts, we have no hope to balance them
	if (freeChainForces) {
		//todo fix this
		//return false;
	}

	//if all joint forces are zero, do an early exit 
	//as zero contact forces are guaranteed to balance the chain
	if (tau.absMax() < 1.0e-3) {
		return 0;
	}

	//check that in matrix D we have 3 rows per contact
	assert( 3*numContacts == D.rows() );

	//summation matrix that adds object wrenches
	//we've moved to full 6D wrenches
	Matrix S(6, 6*numContacts);
	for(int i=0; i<numContacts; i++) {
		S.copySubMatrix(0, 6*i, Matrix::EYE(6,6));
	}

	//quadratic objective matrix Q = S*R*D
	Matrix SR(S.rows(), R.cols());
	matrixMultiply(S, R, SR);
	Matrix Q(S.rows(), D.cols());
	matrixMultiply(SR, D, Q);

	//left-hand equality matrix JTD = JTran * D
	Matrix JTran(J.transposed());
	Matrix JTD(JTran.rows(), D.cols());
	matrixMultiply(JTran, D, JTD);

	//matrix of zeroes for right-hand of friction inequality
	Matrix zeroes(Matrix::ZEROES(F.rows(), 1));

	//matrix of unknowns
	Matrix c_beta(D.cols(), 1);

	//solve QP
	double objVal;
	int result = nonnegativeFactorizedQPSolver(Q, JTD, tau, F, zeroes, c_beta, true, &objVal);
	if (result) {
		if( result > 0) {
			DBGP("Grasp: problem unfeasible");
		} else {
			DBGA("Grasp: QP solver error");
		}
		return result;
	}

	//retrieve contact forces
	Matrix cForces(D.rows(), 1);
	matrixMultiply(D, c_beta, cForces);

	//we now have a contact force vector with 3 entries (force components) per contact
	//all in local contact coordinate systems
	//pass a vector of forces to each chain and let it process it
	numContacts = 0;
	for (int c=0; c<hand->getNumChains(); c++) {
		int chainNumContacts = hand->getChain(c)->getNumContacts(NULL);
		if (!chainNumContacts) continue;
		Matrix chainContactForces(3*chainNumContacts, 1);
		chainContactForces.copySubBlock(0, 0, 3*chainNumContacts, 1, cForces, 3*numContacts, 0);
		//pass it to the chain for processing
		hand->getChain(c)->processComputedStaticForces(chainContactForces, 3);
		numContacts += chainNumContacts;
	}

	//simple sanity check: JT * c = tau
	Matrix fCheck(tau.rows(), 1);
	matrixMultiply(JTran, cForces, fCheck);
	for (int j=0; j<tau.rows(); j++) {
		//I am not sure this works well for universal and ball joints
		double err = fabs(tau.elem(j, 0) - fCheck.elem(j,0));
		//take error as a percentage of desired force, if force is non-zero
		if ( fabs(tau.elem(j,0)) > 1.0e-2) {
			err = err / fabs(tau.elem(j, 0));
		} else {
			//for zero desired torque, out of thin air we pull an error threshold of 1.0e2
			//which is 0.1% of the normal range of torques at 1.0e6
			if (err < 1.0e2) err = 0;
		}
		// 0.1% error is considered too much
		if (  err > 1.0e-1) {
			DBGA("Desired torque not obtained on joint " << j << ", error " << err << 
				" out of " << fabs(tau.elem(j, 0)) );
			return -1;
		}
	}

	//complex sanity check: is object force same as QP optimization result?
	//this is only expected to work if all contacts are on the same object
	double* extWrench = object->getExtWrenchAcc();
	vec3 force(extWrench[0], extWrench[1], extWrench[2]);
	vec3 torque(extWrench[3], extWrench[4], extWrench[5]);
	//take into account the scaling that has taken place
	double wrenchError = objVal*1.0e-6 - (force.len_sq() + torque.len_sq()) * 1.0e6;
	//units here are N * 1.0e-6; errors should be in the range on miliN
	if (wrenchError > 1.0e3) {
		DBGA("Wrench sanity check error: " << wrenchError);
		return -1;
	}
	return 0;
}

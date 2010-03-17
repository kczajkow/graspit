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
// $Id: joint.cpp,v 1.26.2.2 2009/04/27 14:33:13 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Implements the classes: DHTransform, DOF, RevoluteJoint, and PrismaticJoint.
 */

/* standard C includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/draggers/SoRotateDiscDragger.h>

#include "joint.h"
#include "robot.h"
#include "mytools.h"
#include "body.h"
#include "dynJoint.h"
#include "math/matrix.h"

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

/*! Initializes the DHTransform with the 4 DH parameters. */
DHTransform::DHTransform(double thval,double dval,double aval,double alval) :
  theta(thval),d(dval),a(aval),alpha(alval)
{
  transf tr3,tr4;

  dtrans[0] = 0.0;
  dtrans[1] = 0.0;
  dtrans[2] = d;

  atrans[0] = a;
  atrans[1] = 0.0;
  atrans[2] = 0.0;

  tr1 = rotate_transf(theta,vec3(0,0,1));
  tr2 = translate_transf(dtrans);
  tr3 = translate_transf(atrans);
  tr4 = rotate_transf(alpha,vec3(1,0,0));
  tr4TimesTr3 = tr4 * tr3;

  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
  Sets a new d value for prismatic joints and recomputes the current value
  of the transform.
*/
void DHTransform::setD(double q)
{
  d = q;
  dtrans[2] = d;
  tr2 = translate_transf(dtrans);

  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
  Sets a new theta value for revolute joints and recomputes the current value
  of the transform.
*/
void DHTransform::setTheta(double q)
{
  theta = q;
  tr1 = rotate_transf(theta,vec3(0,0,1));
  
  tran = tr4TimesTr3 * tr2 * tr1;
}

/*!
  Unreferences the associated Inventor transform, and deletes the DHTransform
  associated with this joint.
*/
Joint::~Joint()
{
  IVTran->unref(); delete DH;
}

void
Joint::cloneFrom(const Joint *original)
{
	DOFnum  = original->DOFnum;
	jointNum = original->jointNum;
	minVal = original->minVal;
	maxVal = original->maxVal;
	mCouplingRatio = original->mCouplingRatio;
	c = original->c;
	f1 = original->f1;
	f0 = original->f0;
	mK = original->mK;
	mRestVal = original->mRestVal;
	worldAxis = original->worldAxis;
	DH = new DHTransform(original->DH);
	DH->getTran().toSoTransform(IVTran);
	mK = original->mK;
	mRestVal = original->mRestVal;
}

void
Joint::applyPassiveInternalWrenches()
{
	double f = getFriction();
	if (f != 0.0) applyInternalWrench(f);

	f = getSpringForce();
	DBGP("Spring force: " << f);
	applyInternalWrench(-f);
}

/*! Assumes a linear spring with the rest value at the joint minimum */
double
Joint::getSpringForce() const 
{
	return mK * ( getVal() - mRestVal );
}

/*! Given a point that has world transform \a toTarget, this computes the 6x6 Jacobian 
    of this joint relative to that point, in world coordinates */
void
Joint::worldJacobian(transf toTarget, Matrix *J)
{
	//we only care about the transform from joint coordinate system to world coordinate system
	transf myTran = dynJoint->getPrevTrans() * dynJoint->getPrevLink()->getTran();
	transf T = transf(Quaternion::IDENTITY, toTarget.translation()) * myTran.inverse();
	double M[36];
	T.jacobian(M);
	J->copyMatrix(Matrix(M,6,6,true));
}

/*! Given a point that has world transform \a toTarget, this computes the 6x6 Jacobian 
    of this joint relative to that point, in target's local coordinates */
void
Joint::localJacobian(transf toTarget, Matrix *J)
{
	transf myTran = dynJoint->getPrevTrans() * dynJoint->getPrevLink()->getTran();
	transf T = toTarget * myTran.inverse();
	double M[36];
	T.jacobian(M);
	J->copyMatrix(Matrix(M,6,6,true));
}

/*!
  Initializes a prismatic joint from a string read from the robot configuration
  file.  It must also be given pointers to the kinematic chain and robot that
  this joint is a part of.  The format of the string should be: "theta d# a
  alpha minVal maxVal viscousFricionCoeff CoulombFrictionValue", where # is
  the index of the DOF that this joint is hooked to.  It creates a new
  DHTransform using the provided DH values (theta and alpha should be
  expressed in degrees), and creates an Inventor transform.  This returns
  FAILURE if it could not read all the necessary values from the provided
  string, otherwise it returns SUCESS.
*/
int
PrismaticJoint::initJoint(const char *info, int jnum)
{
  char dStr[40],num[40],*tmp;
  double theta,d,a,alpha;
  int argsRead;
  jointNum = jnum;

  argsRead = sscanf(info,"%lf %s %lf %lf %lf %lf %lf %lf %lf %lf",&theta,dStr,&a,
		    &alpha,&minVal,&maxVal,&f1,&f0,&mK, &mRestVal);

  DBGP("theta: " << theta << " dstr: " << dStr << " a: " << a << " alpha: " 
	  << alpha << " minVal: " << minVal << " maxVal: " << maxVal << " f1: " 
	  << f1 << " f0:" << f0 << " mK: " << mK << " mRestVal: " << mRestVal);

  if (argsRead < 6)
    return FAILURE;
  
  if (mK < 0) {
	  DBGA("Negative spring stiffness");
	  return FAILURE;
  } else if (mK>0) {
	  if (mRestVal < minVal || mRestVal > maxVal) {
		  DBGA("Joint spring rest value is not inside legal range");
		  return FAILURE;
	  }
  }
  //convert to graspit force units which for now seem to be the
  //rather strange N * 1.0e6
  mK *= 1.0e6; 

  theta *= M_PI/180.0;
  alpha *= M_PI/180.0;

  d = 0.0;
  tmp = dStr+1;
  sscanf(tmp,"%[0-9]",num);
  DOFnum = atoi(num);
  tmp += strlen(num);

  if (DOFnum > owner->getOwner()->getNumDOF()) {
    pr_error("DOF number is out of range\n");
    return FAILURE;
  }

  if (*tmp=='*') {
    tmp++;
    sscanf(tmp,"%[0-9.-]",num);
    tmp += strlen(num);
    mCouplingRatio = atof(num);
  }
  if (*tmp=='+') {
    tmp++;
    sscanf(tmp,"%lf",&c);
  }

  DH = new DHTransform(theta,d+c,a,alpha);  
  DH->getTran().toSoTransform(IVTran);
  
  return SUCCESS;
}

/*!
  Sets the current joint value to \a q.  The \a d value of the DHTransform
  is then set to \a q + the joint offset \a c. 
*/
int
PrismaticJoint::setVal(double q)
{
  DH->setD(q+c);
  DH->getTran().toSoTransform(IVTran);
  return SUCCESS;  
}

/*!
  Applies equal and opposite forces of magnitude \a f along the axis
  \a worldAxis to the two links connected to this joint.
*/
void
PrismaticJoint::applyInternalWrench(double magnitude)
{
  dynJoint->getPrevLink()->addForce(-magnitude * worldAxis);
  dynJoint->getNextLink()->addForce(magnitude * worldAxis);
}

/*!
  Initializes a revolute joint from a string read from the robot configuration
  file.  It must also be given pointers to the kinematic chain and robot that
  this joint is a part of.  The format of the string should be: "d# d a
  alpha minVal maxVal viscousFricionCoeff CoulombFrictionValue springStiffness 
  restValue", where # is the index of the DOF that this joint is hooked to.  
  It creates a new DHTransform using the provided DH values (alpha should be 
  expressed in degrees), and creates an Inventor transform.  This returns 
  FAILURE if it could not read all the necessary values from the provided 
  string, otherwise it returns SUCESS.
*/
int
RevoluteJoint::initJoint(const char *info, int jnum)
{
  char thStr[40],num[40],*tmp;
  double theta,d,a,alpha;
  int argsRead;
  jointNum = jnum;

  argsRead = sscanf(info,"%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",thStr,&d,&a,
		    &alpha,&minVal,&maxVal,&f1,&f0,&mK, &mRestVal);

  DBGP("thStr: " << thStr << " d: " << d << " a: " << a << " alpha: " 
	  << alpha << " minVal: " << minVal << " maxVal: " << maxVal << " f1: " 
	  << f1 << " f0:" << f0 << " mK: " << mK << " mRestVal: " << mRestVal);

  if (argsRead < 6)
    return FAILURE;

  if (mK < 0) {
	  DBGA("Negative spring stiffness");
	  return FAILURE;
  } else if (mK>0) {
	  if (mRestVal < minVal || mRestVal > maxVal) {
		  DBGA("Joint spring rest value is not within legal range");
		  return FAILURE;
	  }
  }
  //convert to graspit units which for now seem to be the
  //rather strange Nmm * 1.0e6
  mK *= 1.0e6; 
  
  alpha *= M_PI/180.0;
  minVal *= M_PI/180.0;
  maxVal *= M_PI/180.0;

  theta = 0.0;
  tmp = thStr+1;
  sscanf(tmp,"%[0-9]",num);
  DOFnum = atoi(num);
  tmp += strlen(num);

  if (DOFnum > owner->getOwner()->getNumDOF()) {
    pr_error("DOF number is out of range\n");
    return FAILURE;
  }

  if (*tmp=='*') {
    tmp++;
    sscanf(tmp,"%[0-9.-]",num);
    tmp += strlen(num);
    mCouplingRatio = atof(num);
  }
  if (*tmp=='+') {
    tmp++;
    sscanf(tmp,"%lf",&c);
    c *= M_PI/180.0;
  }

  DH = new DHTransform(theta+c,d,a,alpha);  
  DH->getTran().toSoTransform(IVTran);

  return SUCCESS;
}

/*!
  Sets the current joint value to \a q.  The \a theta value of the DHTransform
  is then set to \a q + the joint offset \a c.  
*/
int
RevoluteJoint::setVal(double q)
{
  DH->setTheta(q+c);
  DH->getTran().toSoTransform(IVTran);
  return SUCCESS;  
}

/*!
  Applies equal and opposite torques of magnitude \a f about the axis
  \a worldAxis to the two links connected to this joint.
*/
void
RevoluteJoint::applyInternalWrench(double magnitude)
{
	dynJoint->getPrevLink()->addTorque(-magnitude * worldAxis);
    dynJoint->getNextLink()->addTorque(magnitude * worldAxis);
}

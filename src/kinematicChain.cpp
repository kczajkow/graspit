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
// $Id: kinematicChain.cpp,v 1.24.2.2 2009/04/27 14:33:13 cmatei Exp $
//
//######################################################################

#include "kinematicChain.h"

#include "matvec3D.h"
#include "body.h"
#include "robot.h"
#include "dof.h"
#include "joint.h"
#include "world.h"
#include "matvecIO.h"
#include "dynJoint.h"
#include "humanHand.h"
#include "math/matrix.h"

#ifdef MKL
#include "mkl_wrappers.h"
#else
#include "lapack_wrappers.h"
#endif

//for disp_mat, who needs a new home
#include "maxdet.h"

//#define GRASPITDBG
#include "debug.h"

KinematicChain::KinematicChain(Robot *r,int chainNumber, int jointNum) : owner(r),chainNum(chainNumber),
    numJoints(0),numLinks(0),lastJoint(NULL), IVRoot(NULL),numChildren(0), firstJointNum(jointNum)
{
}

/*!
  The destructor deletes each of the joints in this chain, and asks the
  world to delete each of the links in the chain.  It also detaches any
  connected robots.
*/
KinematicChain::~KinematicChain()
{
  int i;
  IVTran->unref();

  delete [] lastJoint;

  for (i=0;i<numJoints;i++)
    if (jointVec[i]) delete jointVec[i];

    for (i=0;i<numLinks;i++)
      if (linkVec[i]) owner->getWorld()->destroyElement(linkVec[i]);
	

  // go in reverse order becase these operations will delete elements from
  // the children vector
  for (i=numChildren-1;i>=0;i--)
    owner->detachRobot(children[i]);
}

/*! Creates dynamic joints for each of the links in the chain. A dynamic joint
	can be a collection of one or more regular joints, so dynamic joints are
	only created when all the links and the regular joints are already in place.
	The vector of dynamic joint types tells us what kind of dynamic joint
	each link is connected to, and the dynamic joint is then constructed
	based on the appropriate number of regular joints.

	See the DynJoint class for details.
*/
int
KinematicChain::createDynamicJoints(const std::vector<int> &dynJointTypes)
{
	Link* prevLink = owner->getBase();
	for (int l=0; l<numLinks; l++){
		transf dynJointTran = transf::IDENTITY;
		if(l==0) dynJointTran = tran;

		if (dynJointTypes[l] == DynJoint::BALL) {
			linkVec[l]->setDynJoint(new BallDynJoint(jointVec[lastJoint[l]-2],
									jointVec[lastJoint[l]-1],jointVec[lastJoint[l]],
									prevLink,linkVec[l], dynJointTran,
									jointVec[lastJoint[l]]->getTran().inverse()));
			jointVec[lastJoint[l]-2]->dynJoint = linkVec[l]->getDynJoint();
			jointVec[lastJoint[l]-1]->dynJoint = linkVec[l]->getDynJoint();
			jointVec[lastJoint[l]-0]->dynJoint = linkVec[l]->getDynJoint();
		} else if (dynJointTypes[l] == DynJoint::UNIVERSAL) {
			linkVec[l]->setDynJoint(new UniversalDynJoint(jointVec[lastJoint[l]-1],
									jointVec[lastJoint[l]],
									prevLink, linkVec[l], dynJointTran,
									jointVec[lastJoint[l]]->getTran().inverse()));
			jointVec[lastJoint[l]-1]->dynJoint = linkVec[l]->getDynJoint();
			jointVec[lastJoint[l]-0]->dynJoint = linkVec[l]->getDynJoint();
		} else if (dynJointTypes[l] == DynJoint::REVOLUTE) {
			linkVec[l]->setDynJoint(new RevoluteDynJoint(jointVec[lastJoint[l]],
										prevLink,linkVec[l],dynJointTran));
			jointVec[lastJoint[l]]->dynJoint = linkVec[l]->getDynJoint();
		} else if (dynJointTypes[l] == DynJoint::PRISMATIC) {
			linkVec[l]->setDynJoint(new RevoluteDynJoint(jointVec[lastJoint[l]],
									prevLink,linkVec[l],dynJointTran));
			jointVec[lastJoint[l]]->dynJoint = linkVec[l]->getDynJoint();
		} else {
			DBGA("Unknown joint type requested");
			return FAILURE;
		}
		prevLink = linkVec[l];
	}
	return SUCCESS;
}

/*!
  Sets up the chain given a stream from a currently open robot
  configuration file.  It reads the number of joints and the number of links
  and allocates space for those vectors, then it reads in the base transform
  of the chain.  Next, it reads a line for each joint and creates a
  prismatic or revolute joint which are initialized with the kinematic data
  in that line. \a linkDir should be the path to the directory where the link
  body files are kept.
*/ 
int
KinematicChain::initChain(QTextStream &stream,QString &linkDir)
{
	QString line;
	bool ok;

	if (!nextValidLine(&stream, &line)) return FAILURE;
	numJoints = line.toInt(&ok);
	if (!ok || numJoints < 1) {
		DBGA("number of joints < 1");
		return FAILURE;
	}

	if (!nextValidLine(&stream, &line)) {
		DBGA("No more lines after number of joints");
		return FAILURE;
	}

	numLinks = line.toInt(&ok);
	if (!ok || numLinks < 1) {
		DBGA("Number of links < 1");
		return FAILURE;
	}

	jointVec.resize(numJoints, NULL);
	linkVec.resize(numLinks, NULL);  
  
	lastJoint = new int[numLinks];

	IVRoot = new SoSeparator;
	IVTran = new SoTransform;
	IVTran->ref();

	/* read in the finger transformation */
	readTransRotFromQTextStream(stream,tran);
	tran.toSoTransform(IVTran);
	DBGA("  Creating joints");
	numDOF = 0;

	for (int j=0; j<numJoints; j++){
		if (!nextValidLine(&stream, &line)) return FAILURE;
		DBGA("   Joint " << j);

		if (line[0]=='d') jointVec[j] = new RevoluteJoint(this);
		else jointVec[j] = new PrismaticJoint (this);

		if (jointVec[j]->initJoint(line.latin1(), firstJointNum+j) == FAILURE) {
			DBGA("Failed to initialize joint");
			return FAILURE;
		}
	}

	DBGA("  Creating links");
	std::vector<int> dynJointTypes;
	for (int l=0; l<numLinks; l++){
		DBGA("   Link " << l);
		QString jointType;
		if (!nextValidLine(&stream, &line)) {
			DBGA("No line found");
			return FAILURE;
		}
		jointType = line.stripWhiteSpace();
		if (jointType == "Revolute") {
			dynJointTypes.push_back(DynJoint::REVOLUTE);
		} else if (jointType == "Ball") {
			dynJointTypes.push_back(DynJoint::BALL);
		} else if (jointType == "Universal") {
			dynJointTypes.push_back(DynJoint::UNIVERSAL);
		} else if (jointType == "Prismatic") {
			dynJointTypes.push_back(DynJoint::PRISMATIC);
		} else {
			DBGA("Unknown joint type requested");
			return FAILURE;
		}
		if (!nextValidLine(&stream, &line)) {
			DBGA("No line found after joint type");
			return FAILURE;
		}
    	QString linkFilename = line.section(' ',0,0);
		lastJoint[l] = line.section(' ',1,1).toInt(&ok);
		if (!ok || lastJoint[l] < 0 || lastJoint[l] >= numJoints) {
			DBGA("Wrong last joint value: " << lastJoint[l]);
			return FAILURE;
		}
		QString linkName = QString(owner->name()) + QString("_chain%1_link%2").arg(chainNum).arg(l);
		linkVec[l] = new Link(owner,chainNum,l,owner->getWorld(),linkName.latin1());
		if (linkVec[l]->load(linkDir + linkFilename)==FAILURE) {
			delete linkVec[l]; linkVec[l] = NULL;
			DBGA("Failed to load file for link " << l);
			return FAILURE;
		}

		linkVec[l]->addToIvc();
    	IVRoot->addChild(linkVec[l]->getIVRoot());
	}

	DBGA("  Creating dynamic joints");
	createDynamicJoints(dynJointTypes);

	jointsMoved = true;
	updateLinkPoses();
	owner->getWorld()->tendonChange();
	owner->getIVRoot()->addChild(IVRoot);

	return SUCCESS;
}

/*! Copies this chain structure from an existing chain. All joints are
	set to independent copies of the original chain. All links are created
	as clones of the links from the original chain.
*/
void
KinematicChain::cloneFrom(const KinematicChain *original)
{
	IVRoot = new SoSeparator;
	IVTran = new SoTransform;
	IVTran->ref();
	tran = original->getTran();
	tran.toSoTransform(IVTran);
	
	numJoints = original->getNumJoints();
	numLinks = original->getNumLinks();
	jointVec.resize(numJoints,NULL);
	linkVec.resize(numLinks,NULL);  
	lastJoint = new int[numLinks];

	numDOF = 0;
	for (int j=0; j<numJoints; j++){
		if (original->getJoint(j)->getType() == REVOLUTE) {
			jointVec[j] = new RevoluteJoint(this);
		}else if (original->getJoint(j)->getType() == PRISMATIC) {
			jointVec[j] = new PrismaticJoint(this);
		}
		jointVec[j]->cloneFrom( original->getJoint(j) );
	}
	
	std::vector<int> dynJointTypes;
	for (int l=0; l<numLinks; l++){
		lastJoint[l] = original->getLastJoint(l);
		QString linkName =  QString(owner->name())+QString("_chain%1_link%2").arg(chainNum).arg(l);
		linkVec[l] = new Link(owner,chainNum,l,owner->getWorld(),linkName);
		linkVec[l]->cloneFrom( original->getLink(l) );
		//linkVec[l]->setTransparency(0.5);
		IVRoot->addChild(linkVec[l]->getIVRoot());
		dynJointTypes.push_back( original->getLink(l)->getDynJoint()->getType() );
	}
	createDynamicJoints(dynJointTypes);

	jointsMoved = true;
	owner->getIVRoot()->addChild(IVRoot);
}

/*!
  Given a pointer to another robot and an offset transform of the base
  frame of the robot with respect to the end transform of the link of
  this chain.
*/
void
KinematicChain::attachRobot(Robot *r,const transf &offsetTr)
{
  children.push_back(r);
  childOffsetTran.push_back(offsetTr);
  numChildren++;

  if (r->getMountPiece())
    r->getMountPiece()->
      setDynJoint(new FixedDynJoint(linkVec[numLinks-1],r->getMountPiece(), offsetTr));
  else
    r->getBase()->
      setDynJoint(new FixedDynJoint(linkVec[numLinks-1],r->getBase(), offsetTr));
}

/*!
  This separates the robot pointed to by r from this kinematic chain,
  allowing them to move independently.
*/
void
KinematicChain::detachRobot(Robot *r)
{
  int i,j;
  std::vector<transf>::iterator tp;
  std::vector<Robot *>::iterator rp;

  for (i=0,rp=children.begin();rp!=children.end();i++,rp++)
    if (*rp==r) {children.erase(rp); break;}

  for (j=0,tp=childOffsetTran.begin();tp!=childOffsetTran.end();j++,tp++)
    if (j==i) {childOffsetTran.erase(tp); break;}

  numChildren--;
}

/*! Filters a collision report and keeps only collisions of a particular chain
	For now it also throws out collisions between robot parts, as those are 
	giving us problems.
*/
void KinematicChain::filterCollisionReport(CollisionReport &colReport)
{
	//only keep in the collision report those collision that interest this chain
	CollisionReport::iterator it = colReport.begin();
	bool keep;
	while ( it != colReport.end() ) {

		if ( (*it).first->getOwner() != owner ) {
			if ( (*it).second->getOwner() != owner ) {
				keep = false;
			} else {
				if ( ((Link*)(*it).second)->getChainNum() == chainNum )
					keep = true;
				else
					keep = false;
			}
		} else if ( (*it).second->getOwner() != owner ) {
			if ( ((Link*)(*it).first)->getChainNum() == chainNum )
				keep = true;
			else
				keep = false;			
		} else {
			keep = false;
		}
		if (!keep) {
			it = colReport.erase(it);
		} else {
			it++;
		}
	}
}

/*! Reads in the current values of the joints in this chain, and uses
	them to populate the vector \a jointVals. This vector is ordered to
	contain all the joints of the robot (not only of this chain) and
	joints are indexed by their number in the robot structure, not in
	this chain's struture.
*/
void
KinematicChain::getJointValues(double *jointVals) const
{
	for (int j=0; j<numJoints; j++) {
		jointVals[firstJointNum + j] = jointVec[j]->getVal();
	}
}

/*! Sets the current joint values from the vector \a jointVals. This 
	vector is ordered to contain all the joints of the robot (not only 
	of this chain) and joints are indexed by their number in the robot 
	structure, not in this chain's struture.
*/
void
KinematicChain::setJointValues(const double *jointVals)
{
	for (int j=0; j<numJoints; j++) {
		jointVec[j]->setVal( jointVals[firstJointNum + j] );
	}
}

/*! Computes forward kinematics for the current joint values, then
	sets the link transforms accordingly. This is the only way that 
	robot links should ever be moved (the robot asks the dof' for
	joint values, tells the chains to set those values then tells
	the chains to update link poses).
*/
void
KinematicChain::updateLinkPoses()
{
	std::vector<transf> newLinkTranVec;
	newLinkTranVec.reserve(numLinks);
	fwdKinematics(NULL, newLinkTranVec);

	for (int l=0;l<numLinks;l++) {
		linkVec[l]->setTran(newLinkTranVec[l]);
	}

	for (int j=0;j<numChildren;j++) {
		children[j]->simpleSetTran(childOffsetTran[j]*newLinkTranVec[numLinks-1]);
	}

	if ( owner->isA("HumanHand") ){
		((HumanHand*)owner)->updateTendonGeometry();
		owner->getWorld()->tendonChange();
	}
}


/*!
  Given a array of joint values for each joint in the chain, this method
  will compute the transforms for each link in the chain with respect to
  world coordinates.  It does not affect the chain itself.
*/
void
KinematicChain::fwdKinematics(const double *jointVals, std::vector<transf> &newLinkTranVec) const
{
	transf total = tran * owner->getTran();
	int l=0;
	for (int j=0;j<numJoints;j++) {    
		if (!jointVals) {
			total = jointVec[j]->getTran( jointVec[j]->getVal() ) * total;
		} else {
			total = jointVec[j]->getTran( jointVals[firstJointNum + j] ) * total;
		}
		if (l<numLinks && lastJoint[l]==j) {
			newLinkTranVec[l] = total;
			l++;
		}
	}
}

/*! Given an array of desired joint values, this computed an infinitesimal motion
	of each link as motion *from the current values* towards the desired values is 
	started. Used mainly to	see if any contacts prevent this motion.
*/
void
KinematicChain::infinitesimalMotion(const double *jointVals, std::vector<transf> &newLinkTranVec) const
{
	//start with the link jacobian in local link coordinates
	//but keep just the actuated part
	Matrix J(actuatedJacobian(linkJacobian(false)));
	//joint values matrix
	Matrix theta(numJoints, 1);
	//a very small motion
	//either 0.1 radians or 0.1 mm, should be small enough
	double inf = 0.1;
	//a very small threshold
	double eps = 1.0e-6;
	for(int j=0; j<numJoints; j++) {
		int sign;
		if ( jointVals[firstJointNum + j] + eps < jointVec[j]->getVal() ) {
			sign = -1;
		} else if ( jointVals[firstJointNum + j] > jointVec[j]->getVal() + eps ) {
			sign = 1;
		} else {
			sign = 0;
		}
		theta.elem(j,0) = sign * inf;
	}
	//compute infinitesimal motion
	Matrix dm(6*numLinks, 1);
	matrixMultiply(J, theta, dm);
	//and convert it to transforms
	for (int l=0; l<numLinks; l++) {
		transf tr = rotXYZ( dm.elem(6*l+3, 0), dm.elem(6*l+4, 0), dm.elem(6*l+5, 0) ) * 
					translate_transf( vec3( dm.elem(6*l+0, 0), dm.elem(6*l+1, 0), dm.elem(6*l+2, 0) ) );
		newLinkTranVec[l] = tr;
	}
}

void 
KinematicChain::getDynamicJoints(std::vector<DynJoint*> *dj) const
{
	DynJoint *lastDynJoint = NULL;
	for(int j=0; j<numJoints; j++) {
		if (jointVec[j]->dynJoint == lastDynJoint) continue;
		lastDynJoint = jointVec[j]->dynJoint;
		dj->push_back(lastDynJoint);
	}
}

/*! The link jacobian relates joint angle changes to link movement.
	Its transpose relatef forces applied to link to forces applied
	to joints. If \a worldCoords is false, the jacobian is computed
	in each link's coordinate system.
*/
Matrix
KinematicChain::linkJacobian(bool worldCoords) const
{
	Matrix J(Matrix::ZEROES(numLinks * 6, numLinks * 6));
	Matrix indJ(6,6);
	for (int l=0; l<numLinks; l++) {
		int jointCount = 0;
		DynJoint *lastDynJoint = NULL;
		for(int j=0; j<=getLastJoint(l); j++) {
			//only joints in the dynamic sense
			if (jointVec[j]->dynJoint == lastDynJoint) continue;
			lastDynJoint = jointVec[j]->dynJoint;
			//create the individual jacobian
			if (worldCoords) {
				jointVec[j]->worldJacobian(linkVec[l]->getTran(), &indJ);
			} else {
				jointVec[j]->localJacobian(linkVec[l]->getTran(), &indJ);
			}
			//set as a block of the big one
			J.copySubMatrix(6*l, 6*jointCount, indJ);
			jointCount++;
		}
	}
	return J;
}

/*! The contact jacobian relates joint angle changes to motion at the
	points of contact. Its transpose relates contact forces to joint
	forces.
*/
Matrix
KinematicChain::contactJacobian(bool worldCoords)
{
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		numContacts += linkVec[l]->getNumContacts();
	}
	if (!numContacts) {
		DBGP("Contact Jac requested, but no contacts!");
		return Matrix(0,0);
	}
	Matrix CJ(Matrix::ZEROES(6*numContacts, 6*numLinks));
	Matrix indCJ(6,6);
	int contactCount = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			Contact *contact = *it;
			int jointCount = 0;
			DynJoint *lastDynJoint = NULL;
			for(int j=0; j<=getLastJoint(l); j++) {
				//only joints in the dynamic sense
				if (jointVec[j]->dynJoint == lastDynJoint) continue;
				lastDynJoint = jointVec[j]->dynJoint;
				//create the individual contact jacobian
				transf contactTran = contact->getContactFrame() * linkVec[l]->getTran();
				if (worldCoords) {
					jointVec[j]->worldJacobian(contactTran, &indCJ);
				} else {
					jointVec[j]->localJacobian(contactTran, &indCJ);
				}
				//set as a block of the big one
				CJ.copySubMatrix(6*contactCount, 6*jointCount, indCJ);
				jointCount++;
			}
			contactCount++;
		}
	}
	return CJ;
}

/*! The active link jacobian will build the regular link Jacobian, then
	only keep the rows that correspond to links that have at least one
	contact (as links with no contacts can not external forces applied
	to them).
*/
Matrix
KinematicChain::activeLinkJacobian(bool worldCoords)
{
	Matrix J(linkJacobian(worldCoords));
	if (!J.rows() || !J.cols()) return Matrix(0,0);
	int activeLinks = 0;
	for (int l=0; l<numLinks; l++) {
		if (linkVec[l]->getNumContacts()) {
			activeLinks++;
		}
	}
	if (!activeLinks) {
		DBGA("Active link Jac requested, but no active links!");
		return Matrix::ZEROES(0,0);
	}
	int activeRows = 6*activeLinks;
	Matrix activeJ(activeRows, J.cols());
	int linkCount = 0;
	//only keep rows that correspond to links that have at least one contact
	for (int l=0; l<numLinks; l++) {
		if (!linkVec[l]->getNumContacts()) continue;
		activeJ.copySubBlock(6*linkCount, 0, 6, J.cols(), J, 6*l, 0);
		linkCount++;
	}
	return activeJ;
}

/*! Build the full contact jacobian, then only keeps some of the rows.
	For now, we only keep the 3 rows that in the transpose Jacobian
	correspond to the force component of the contact wrench. In the 
	future, we should also keep the torsional friction component,
	for soft contacts.
*/
Matrix 
KinematicChain::activeContactJacobian(bool worldCoords)
{
	Matrix J(contactJacobian(worldCoords));
	if (!J.rows() || !J.cols()) return Matrix(0,0);
	int numContacts = J.rows() / 6;
	//make sure this was a multiple of 6
	assert( 6*numContacts == J.rows() );

	//keep all 3 dof of force
	Matrix activeJ(3*numContacts, J.cols());
	for (int c=0; c<numContacts; c++) {
		activeJ.copySubBlock(3*c, 0, 3, J.cols(), J, 6*c, 0);
	}
	return activeJ;
}

/*! This function takes a Jacobian with all columns (6dof per joint
	and returns a version that has only those columns that correspond
	to the actuated dofs in each joint (e.g. the z axis for revolute 
	joints) 
*/
Matrix
KinematicChain::actuatedJacobian(const Matrix &fullColumnJ) const
{
	std::vector<DynJoint*> dynJoints;
	getDynamicJoints(&dynJoints);
	int activeRows = fullColumnJ.rows();
	if (!activeRows) return Matrix(0,0);
	//first count the constraints
	int numConstrained = 0;
	int numActuated = 0;
	for(int dj=0; dj<(int)dynJoints.size(); dj++) {
		numConstrained += dynJoints[dj]->getNumConstraints();
		numActuated += 6 - dynJoints[dj]->getNumConstraints();
	}
	assert(numActuated + numConstrained == 6*numLinks);
	assert(fullColumnJ.cols() == 6*numLinks);
	Matrix blockJ(activeRows, numActuated);
	//then copy only actuated columns in the block jacobian.
	int actIndex = 0;
	for(int dj=0; dj<(int)dynJoints.size(); dj++) {
		char constraints[6];
		dynJoints[dj]->getConstraints(constraints);
		for (int c=0; c<6; c++) {
			if (!constraints[c]) {
				//actuated direction
				blockJ.copySubBlock(0, actIndex, activeRows, 1, fullColumnJ, 0, 6*dj+c);
				actIndex++;
			}
		}
	}
	assert(actIndex == numActuated);
	return blockJ;
}

/*! Given a set of joint torques, performs a quasi-static force balance
	computation to compute the contact forces that balance the system.
	It is possible that no such forces exist, in which case the contacts
	will slip. Legal contact forces that balance the fingers might not
	balance the object too - the resultant might be an unbalanced force.
	After this is done, the unbalanced object force can be computes as
	well.

	Return codes: 0 is success, >0 means finger slip, no legal contact
	forces exist; <0 means error in computation 
*/
int KinematicChain::accumulateStaticLinkForcesWithJacobian(const Matrix &jointTorques)
{
	//count contacts
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		numContacts += getLink(l)->getNumContacts();
	}
	if (!numContacts) {
		Matrix foobar(jointTorquesVector(jointTorques));
		double freeForce = foobar.absMax();
		if (freeForce > 1.0) {
			DBGA("Chain: no contact and free force " << freeForce);
		}
		return 0;
	}
	FILE *fp = NULL;
	//fp = fopen("jacobian.txt","w");

	//prepare a list of the dynamic joints in this chain
	std::vector<DynJoint*> dynJoints;
	getDynamicJoints(&dynJoints);

	//contact Jacobian is now used in local contact coordinates
	Matrix J(contactJacobian(false));
	Matrix activeJ(activeContactJacobian(false));
	if (fp) {
		fprintf(fp,"\nOriginal Jacobian:\n");
		J.print(fp);
		fprintf(fp,"\nActive Jacobian:\n");
		activeJ.print(fp);
	}

	int activeRows = activeJ.rows();
	if (!activeRows) {
		DBGA("Error: Jacobian is empty!");
		if (fp) fclose(fp);
		return -1;
	}

	//keep only actuated columns
	Matrix blockJ( actuatedJacobian(activeJ) );
	if (fp) {
		fprintf(fp,"\nBlock jacobian:\n");
		blockJ.print(fp);
	}
	Matrix blockJTran(blockJ.transposed());

	//prepare right-hand side
	Matrix tau(jointTorquesVector(jointTorques));
	if (fp) {
		fprintf(fp,"\ntau matrix:\n");
		tau.print(fp);
	}

	//if all joint forces are zero, do an early exit 
	//as zero contact forces are guaranteed to balance the chain
	if (tau.absMax() < 1.0e-3) {
		if (fp) {
			fprintf(fp,"Zero joint forces, early exit\n");
			fclose(fp);
		}
		return 0;
	}

	//attempt to solve the underdetermined system
	Matrix sol(activeRows, 1);
	int result = solveContactForcesQP(blockJTran, tau, sol, fp);
	//int result = underDeterminedSolveQR(blockJTran, tau, sol);
	if (result) {
		//result < 0 means an error has happened
		if (result < 0) {
			DBGA("Constraint solve error, result: " << result);
		}
		//result > 0 simply means the system has no solution, chain is unbalanced
		if (fp) fclose(fp);
		return result;
	}
	if (fp) {
		fprintf(fp, "\nUndet solve result: %d\n",result);
		fprintf(fp, "\nSolution:\n");
		sol.print(fp);
		fprintf(fp,"Norm: %e\n",sol.fnorm());
	}

	//we have the result forces. Copy them into their own full matrix
	//of same size as original Jacobian J. Pad with zeroes for the inactive rows
	//this part must know about how the activeJ is derived from the J
	//therefore it is really not modular
	Matrix f(Matrix::ZEROES(J.rows(), 1));
	int count = 0;
	for(int l=0; l<numLinks; l++) {
		if (!linkVec[l]->getNumContacts()) continue;
		for (int c=0; c<linkVec[l]->getNumContacts(); c++) {
			//assume we only kept first 3 dofs from each contact
			//the original J has nothing for links with no contacts
			f.copySubBlock(6*count, 0, 3, 1, sol, 3*count, 0);
			count++;
		}
	}

	//add the visual markers
	//todo fix this hack
	processComputedStaticForces(sol, 3);

	//from here on it's all debug and sanity checks. Some checks are done only
	//if we have a file to write to, some are done always.

	if (fp) {
		//do a sanity check
		Matrix lCheck(numJoints, 1);
		matrixMultiply(blockJTran, sol, lCheck);
		fprintf(fp,"\nSystem check lCheck:\n");
		lCheck.print(fp);
		fprintf(fp,"Forces matrix:\n");
		f.print(fp);
	}

	//do another sanity check
	Matrix JTran(J.transposed());
	Matrix fCheck(6*numLinks, 1);
	matrixMultiply(JTran, f, fCheck);
	if (fp) {
		fprintf(fp, "Recovered joint forces:\n");
		fCheck.print(fp);
	}
	for (int j=0; j<numJoints; j++) {
		//I am not sure this works well for universal and ball joints
		double err = fabs(jointTorques.elem(jointVec[j]->getNum(), 0) - fCheck.elem(6*j+5,0));
		//take error as a percentage of desired force, if force is non-zero
		if ( fabs(jointTorques.elem(jointVec[j]->getNum(),0)) > 1.0e-2) {
			err = err / fabs(jointTorques.elem(jointVec[j]->getNum(), 0));
		} else {
			//for zero desired torque, out of thin air we pull an error threshold of 1.0
			if (err < 1.0) err = 0;
		}
		// 0.1% error is considered too much
		if (  err > 1.0e-1) {
			DBGA("Desired torque not obtained on joint " << j << ", error " << err << 
				" out of " << fabs(jointTorques.elem(jointVec[j]->getNum(), 0)) );
			if (fp) fprintf(fp,"Desired torque failed on joint %d: %f, error is %f\n",
							j, fCheck.elem(6*j+5,0), err);
			result = -1;
		}
	}
	if (fp) {
		fclose(fp);
	}
	return result;
}

/*! Takes all the contact forces that have been pre-set (presumably by dynamics),
	converts them to world coords and adds them up in a resultant force. There is 
	overlap	with processComputedStaticForces, which sets contact forces differently,
	but then adds them up as well.
	todo: unify this into a good architecture
*/
vec3 
KinematicChain::contactForceResultant()
{
	//todo this does not distinguish between contact with a certain object
	//and self-contacts
	vec3 resultant(0,0,0);
	for(int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for (it=contacts.begin(); it!=contacts.end(); it++) {
			//we want the forces as applied to external objects
			Contact *contact = (*it)->getMate();
			assert(contact);
			//get the dynamic force
			double *cW = contact->getDynamicContactWrench();
			vec3 contactForce(cW[0], cW[1], cW[2]);
			//force is in contact coordinate system, convert to world
			transf contactTran = contact->getContactFrame() * contact->getBody1()->getTran();
			contactForce = contactForce * contactTran;
			resultant = resultant + contactForce;
		}
	}
	return resultant;
}

/*! Given a matrix with all the joint torques of the robot, numbered as in the 
	the robot scheme, this extracts a vector of chain joint torques, in the order
	in which they appear in this chain
*/
Matrix 
KinematicChain::jointTorquesVector(Matrix fullRobotTorques)
{
	assert(fullRobotTorques.cols() == 1);
	//TODO make this general for non-revolute joints
	Matrix tau(numJoints,1);
	for (int j=0; j<numJoints; j++) {
		tau.elem(j,0) = fullRobotTorques.elem(jointVec[j]->getNum(), 0);
	}
	return tau;
}

/*! Returns the number of contacts between the links of this chain and the object
	\a body. If \a body == NULL, returns the total number of contacts, regardless
	of what object they are against.
	Todo: what about self-colision?
*/

int 
KinematicChain::getNumContacts(Body *body)
{
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		numContacts += linkVec[l]->getNumContacts(body);
	}
	return numContacts;
}

/*! Computes the transformation matrix that multiplies friction amplitudes by friction
	wrench direction and adds normal force amplitudes */
Matrix
KinematicChain::frictionForcesMatrix()
{
	//first we need to count how many total friction wrenches per contact we have
	//this is replicated from solveWithQP for now
	int numFrictionWrenches = 0;
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			numFrictionWrenches += (*it)->numFrictionEdges;
			numContacts++;
		}
	}
	if (!numContacts || !numFrictionWrenches) return Matrix(0,0);
	//padded with zeroes initially
	Matrix Rf(Matrix::ZEROES(3*numContacts, numContacts + numFrictionWrenches));
	//loop again through contacts and create individual transform matrix
	int contactCount = 0;
	int wrenchCount = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			Contact *contact = *it;
			Matrix Rfi(3, contact->numFrictionEdges + 1);
			//the column for the normal force
			Rfi.elem(0,0) = Rfi.elem(1,0) = 0.0; Rfi.elem(2,0) = 1.0;
			//the columns for the friction edges
			//for now we only include frictional force
			for(int edge=0; edge<contact->numFrictionEdges; edge++) {
				Rfi.elem(0, edge+1) = contact->frictionEdges[6*edge+0];
				Rfi.elem(1, edge+1) = contact->frictionEdges[6*edge+1];
				Rfi.elem(2, edge+1) = contact->frictionEdges[6*edge+2];
			}
			//flip the sign of Rfi to get contact pointing in the right direction
			Rfi.multiply(-1.0);
			Rf.copySubMatrix(3*contactCount, contactCount + wrenchCount, Rfi);
			wrenchCount += contact->numFrictionEdges;
			contactCount++;
		}
	}
	return Rf;
}

/*! Computes inequality constraint matrix that compares normal force 
	and friction for each contact */
Matrix
KinematicChain::frictionConstraintsMatrix()
{
	//first we need to count how many total friction wrenches per contact we have
	//this is replicated in way too many places...
	int numFrictionWrenches = 0;
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			numFrictionWrenches += (*it)->numFrictionEdges;
			numContacts++;
		}
	}
	if (!numContacts || !numFrictionWrenches) return Matrix(0,0);
	Matrix F(Matrix::ZEROES(numContacts, numContacts + numFrictionWrenches));
	//go through the contacts yet again
	int contactCount = 0;
	int wrenchCount = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			Contact *contact = *it;
			F.elem(contactCount, contactCount + wrenchCount) = -1.0 * contact->getCof();
			for (int edge=0; edge < contact->numFrictionEdges; edge++) {
				F.elem(contactCount, contactCount + wrenchCount + 1 + edge) = 1.0;
			}
			wrenchCount += contact->numFrictionEdges;
			contactCount++;
		}
	}
	return F;
}

/*! Builds the matrix that transforms forces in contact frame to wrenches applied on the 
	other object, expressed in world coordinates.
*/
Matrix
KinematicChain::contactForcesToWorldWrenches()
{
	int numContacts = 0;
	for (int l=0; l<numLinks; l++) {
		numContacts += linkVec[l]->getContacts().size();
	}
	if (!numContacts) return Matrix(0,0);

	Matrix Ro(Matrix::ZEROES(6*numContacts, 3*numContacts));

	int contactCount = 0;
	for (int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for ( it = contacts.begin(); it!=contacts.end(); it++) {
			transf contactTran = (*it)->getContactFrame() * linkVec[l]->getTran();
			//the force transform is simple, just the matrix that changes coord. systems
			mat3 R; contactTran.rotation().ToRotationMatrix(R);
			Ro.copySubMatrix(6*contactCount, 3*contactCount, Matrix::ROTATION(R));
			//for torque we also multiply by a cross product matrix
			vec3 worldLocation = contactTran.translation();
			vec3 cog = (*it)->getBody2()->getTran().translation();
			mat3 C; C.setCrossProductMatrix(worldLocation - cog); 
			Matrix CR(3,3);
			matrixMultiply(Matrix::ROTATION(C.transpose()), Matrix::ROTATION(R), CR);
			//also scale by object max radius so we get same units as force
			//and optimization does not favor torque over force
			double scale = 1.0;
			if ((*it)->getBody2()->isA("GraspableBody")) {
				scale = scale / static_cast<GraspableBody*>((*it)->getBody2())->getMaxRadius();
			}
			CR.multiply(scale);
			Ro.copySubMatrix(6*contactCount+3, 3*contactCount, CR);
			contactCount++;
		}
	}
	return Ro;
}

/*! Given its own set of contact forces, in contact coords, it converts them to 
	world coords and adds them to the object accumulator, and also sets them for
	rendering purposes. Replicates the same thing from computeStatic..., but without 
	the sanity checks; just sets the values.
	Input can be in the form of 3 entries per contact (just force) or 6 entries. 
	Currently, torque entries are ignored even when present.
	todo: resolve the duplication from above
*/
void 
KinematicChain::processComputedStaticForces(Matrix f, int numEntriesPerContact)
{
	//first transform contact force to world coords forces / torques
	Matrix Ro(contactForcesToWorldWrenches());
	//todo generalize this
	assert(numEntriesPerContact == 3);
	Matrix oW(Ro.rows(), 1);
	matrixMultiply(Ro, f, oW);

	int count=0;
	int ne = numEntriesPerContact;
	assert(ne==3 || ne==6);
	for(int l=0; l<numLinks; l++) {
		std::list<Contact*> contacts = linkVec[l]->getContacts();
		std::list<Contact*>::iterator it;
		for (it=contacts.begin(); it!=contacts.end(); it++) {
			Contact *contact = *it;
			transf contactTran = contact->getContactFrame() * linkVec[l]->getTran();
			//contact force and torque it produces
			//both in world coordinates
			int sz = 6;

			vec3 force(oW.elem(sz*count+0,0), oW.elem(sz*count+1,0), oW.elem(sz*count+2,0));
			vec3 torque(oW.elem(sz*count+3,0), oW.elem(sz*count+4,0), oW.elem(sz*count+5,0));

			count++;
			//accumulate them on object and put visual marker on contact mate
			if (contact->getBody2()->isDynamic()) {
				DynamicBody *object = (DynamicBody*)(contact->getBody2());
				//compute force and torque in external body reference frame
				//and scale them down for now for rendering and output purposes
				force = 1.0e-6 * force * object->getTran().inverse();
				//accumulate them on object
				object->addForce(force);
				torque = 1.0e-6 * torque * object->getTran().inverse();
				//also scale torque by object max radius to get to same units as force
				//torque = (1.0 / object->getMaxRadius()) * torque;
				//scaling now done in conversion matrix
				object->addTorque(torque);
				//accumulate force on contact 
				contact->getMate()->setDynamicContactForce(force * contact->getMate()->getContactFrame().inverse());
			}
		}
	}
}


/*! Given a vector of desired joint forces and a block Jacobian with only the columns
	for the actuated dofs and only the rows for the admissible contact dofs, this 
	attempts to compute the contact forces that balance the system. Taking into account
	friction constraints, attempts to cast the problem as a Quadratic Program
*/
int 
KinematicChain::solveContactForcesQP(Matrix &blockJTran, Matrix &tau, Matrix &c, FILE *fp)
{
	//the matrix that relates friction wrench amplitudes to contact forces
	Matrix D(frictionForcesMatrix());
	if (fp) {
		fprintf(fp,"\nRf matrix:\n");
		D.print(fp);
	}

	//the unknowns matrix
	Matrix c_beta(D.cols(), 1);

	//prepare equality constraint matrix: Eq = JT * D
	Matrix Eq(blockJTran.rows(), D.cols());
	matrixMultiply(blockJTran, D, Eq);

	//the matrix the encapsulated friction constraints inequalities
	Matrix InEq(frictionConstraintsMatrix());
	if (fp) {
		fprintf(fp,"\nInEq matrix:\n");
		InEq.print(fp);
	}
	//InEq * c_beta <= 0
	Matrix z(Matrix::ZEROES(InEq.rows(), 1));

	//matrix that transforms contact forces to world coord system
	Matrix Ro(contactForcesToWorldWrenches());
	if (fp) {
		fprintf(fp,"\nRo matrix:\n");
		Ro.print(fp);
	}

	//build summation matrix that adds together multiple forces, all in object coords
	int numContacts = InEq.rows();
	Matrix S(6, 6*numContacts);
	int sz = 6;
	for (int i=0; i<numContacts; i++) {
		S.copySubMatrix(0, sz*i, Matrix::EYE(sz,sz));
	}
	if (fp) {
		fprintf(fp,"\nS matrix:\n");
		S.print(fp);
	}

	//chain matrices together: Q = S * Ro * D
	Matrix SRo(S.rows(), Ro.cols());
	matrixMultiply(S, Ro, SRo);
	if (fp) {
		fprintf(fp,"\nS*Ro matrix:\n");
		SRo.print(fp);
	}
	Matrix Q(S.rows(), D.cols());
	matrixMultiply(SRo, D, Q);
	if (fp) {
		fprintf(fp,"\nS*Ro*D = Q matrix:\n");
		Q.print(fp);
	}

	//minimization matrix is QT * Q
	Matrix QTQ(Q.cols(), Q.cols());
	matrixMultiply(Q.transposed(), Q, QTQ);
	if (fp) {
		fprintf(fp,"\nQTQ matrix:\n");
		QTQ.print(fp);
		fprintf(fp,"Rank: %d\n",QTQ.rank());
	}

	//we have Quadratic Program:
	// minimize cT * QTQ * c
	// subject to:
	//  c>=0
	//  Eq * c = tau
	//  InEq * c <= 0
	// QTQ is positive semidefinite by construction 
	// (as a product of a matrix and its transpose)
	// now solve the QP
	// perform scaling before solving as needed
	Matrix scaledTau(tau);
	double objVal;
	int result = nonnegativeFactorizedQPSolver(Q, Eq, tau, InEq, z, c_beta, true, &objVal);
	if (fp) {
		fprintf(fp,"QP solver result: %d and optimized value: %f\n",result, objVal);
	}
	DBGP("QP objective: " << objVal);
	//----------mosek debug-----------------
	/*
	FILE *mfp = fopen("mosek.txt","w");
	fprintf(mfp,"Problem: minimize xT * Q * x subject to:\n");
	fprintf(mfp,"A*x = t\n");
	fprintf(mfp,"b*x <= 0\n");
	fprintf(mfp,"all unknowns (xi) >= 0\n");
	fprintf(mfp, "\nA matrix:\n");
	Eq.print(mfp);
	fprintf(mfp, "\nt vector:\n");
	scaledTau.print(mfp);
	fprintf(mfp,"b vector:\n");
	InEq.print(mfp);
	fprintf(mfp,"Q matrix:\n");
	QTQ.print(mfp);
	fprintf(mfp,"Note that Q = FT * F where F is:\n");
	Q.print(mfp);
	if(result>0) {
		fprintf(mfp,"Mosek optimization returns MSK_RES_OK, and correctly reports problem is unfeasible\n");
	} else if (result<0) {
		fprintf(mfp,"Mosek optimization returns error code 4000. If max number of iterations is increased to 800, Mosek returns error code 4006\n");
	}
	fclose(mfp);
	*/
	//--------------------------------------

	//as debug, we can also try a simple undetermined solve on the same system
	/*
	if (result){
		DBGA("QP fails, trying underdetermined");
		result = underDeterminedSolveQR(Eq, tau, c_beta);
		if (!result) {
			DBGA("System solve successful");
			if (fp) {
				fprintf(fp,"System solve result:\n");
				c_beta.print(fp);
			}
		} else {
			DBGA("System solve fails as well");
		}
	}
	*/
	if (result < 0) {
		DBGA("QP solver error!");
		return -1;
	} else if (result > 0) {
		DBGP("QP unfeasible.");
		return 1;
	}
	DBGP("Solver success");
	//recover contact forces
	matrixMultiply(D, c_beta, c);
	return 0;
}

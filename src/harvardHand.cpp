//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2004  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under an Research and Educational Use
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
// Author: Akshay Pundle
//
//######################################################################

/*! \file 
  \brief Implements the %HarvardHand hand class, a specialized hand subclass
 */

#include "harvardHand.h"
#include "world.h"
#include <iostream>
using namespace std;

#define AUTO_GRASP_TIME_STEP 0.01

#define BADCONFIG                                                   \
{                                                                   \
  QTWARNING("There was a problem reading the configuration file."); \
  file.close();                                                     \
  return FAILURE;                                                   \
}

HarvardHandDOF::HarvardHandDOF(const DOF &oldDOF) : DOF(oldDOF)
{
	reconnectAts.resize(owner->getNumChains(), std::vector<double>() );
	for(int i=0;i<owner->getNumChains();i++)
		reconnectAts[i].resize(owner->getChain(i)->getNumJoints() );
}


//  std::list<Joint *> jointList;
int
HarvardHandDOF::setVal(double q1)
{
  // for reconnecting after breakaway
  if (!tempList.empty() && (q1<reconnectAt || q1==minq) )
	reconnect();	
  
  DOF::setVal(q1);

  return SUCCESS;
}

void
HarvardHandDOF::breakAway()
{
  tempList = jointList;
  jointList.erase(jointList.begin());
  reconnectAt = q;
}

void
HarvardHandDOF::reconnect()
{
  jointList = tempList;
  tempList.clear();
}


int
HarvardHand::load(QString filename)
{
  DOF *tmpDOF;
  if (Robot::load(filename) == FAILURE) return FAILURE;

  //! make all DOF's point to HarvardHandDOF
  for(int i=0;i<Robot::getNumDOF();i++)
  {
	  tmpDOF = dofVec[i];
      dofVec[i] = new HarvardHandDOF(*tmpDOF);
      delete tmpDOF;
  }
  return SUCCESS;
}

void
HarvardHand::fwdKinematics(double* dofVals,std::vector<transf>& trVec,int chainNum)
{
  int numJoints = chainVec[chainNum]->getNumJoints();
  double *jointVals = new double[numJoints];
  Joint *joint;

  for (int j=0;j<numJoints;j++) {
    joint = chainVec[chainNum]->getJoint(j);

	// all joints except first can be in breakaway
    if (j>=0 &&	((HarvardHandDOF *)dofVec[joint->getDOFNum()])->inBreakAway())
      jointVals[j] = joint->getVal();
    else 
      jointVals[j] = dofVals[joint->getDOFNum()] * joint->getRatio();
  }

  chainVec[chainNum]->fwdKinematics(jointVals,trVec);
  delete [] jointVals;
}

bool 
HarvardHand::linkContactsPreventDOFMotion(int c, int l, const transf& motion)
{
	if ( !chainVec[c]->getLink(l)->contactsPreventMotion(motion) ){
		return false;
	}
	int d = c+1;
	if (breakAway(c,l,d)) {
		return false;
	}
	return true;
}

bool HarvardHand::breakAway(int f, int l, int d)
{
	if ( !((HarvardHandDOF *)dofVec[d])->inBreakAway()  && dofVec[d]->getVal() < dofVec[d]->getMax() ) 
		{
		((HarvardHandDOF *)dofVec[d])->breakAway();
		getChain(f)->getLink(l)->setMoving(true);
		//getChain(f)->getLink(l+1)->setMoving(true);
		return true;
		}

	return false;
}

ColReportT
HarvardHand::forceDOFTo(double *desiredDOFVals, bool fastCollision)
{
  ColReportT colReport,result;
  int numCols;
  bool done=true;
  int f=-1,l=-1;
  int i,d;

  do {
    done = true;
    colReport = Robot::forceDOFTo(desiredDOFVals,fastCollision);
    numCols = colReport.size();

    result.insert(result.end(),colReport.begin(),colReport.end());

    for (i=0;i<numCols;i++) {
		if (colReport[i].first->inherits("Link") && ((Link *)colReport[i].first)->getOwner() == this) {

			f = ((Link *)colReport[i].first)->getChainNum();
			l = ((Link *)colReport[i].first)->getLinkNum();
			d = (chainVec[f])->getJoint(l)->getDOFNum();
			cerr << " f : "<<f <<" l : " << l <<" d : "<<d<<"\n";
			if (breakAway(f,l,d)) done = false;
       
		}
		if (colReport[i].second->inherits("Link") && ((Link *)colReport[i].second)->getOwner() == this) {
			f = ((Link *)colReport[i].second)->getChainNum();
			l = ((Link *)colReport[i].second)->getLinkNum();
			
			d= (chainVec[f])->getJoint(l)->getDOFNum();
			cerr << " f : "<<f <<" l : " << l <<" d : "<<d<<"\n";
			if (breakAway(f,l,d)) done = false;
		}
	}

  }while(!done);

  return result;
}



double HarvardHand::current_unbalanced_force(double tmp_dof)
{

	vec3 u_force(0,0,0);

	// angle to torque ratio, the torque ratios between the 
	// 2 torques are t2 : t1 :: 10 : 1
	double Tr1=20;
	double Tr2=200;

	// stiffness values for joint
	// k2 : k1 :: 10 : 1
	// k2/k1 : t2/t1 :: 1 : 1
	double k1=20,k2=200;

	//skin stiffness
	double ks=1000*k1;

	//rest angles
	double phi1=deg2rad(45);
	double phi2=deg2rad(65);

	double linkl=70; // link length (get this from the robot desc)

	// calculate forces at all other contacts once
	for(int l=0;l<4;l++){	
		cerr << "contact 1 " << contacts[2*l].init <<" contact 2 " << contacts[2*l+1].init <<"\n";
		cerr << "a1 " << contacts[2*l].a<<" a2 " << contacts[2*l+1].a<<"\n";
		// contact on link1 and link 2

		if(contacts[2*l].init && contacts[2*l].a>0 && contacts[2*l+1].init && contacts[2*l+1].a>0 )	{
			vec3 fn1(0,0,0);
			vec3 fn2(0,0,0);
			vec3 ft1(0,0,0);
			vec3 ft2(0,0,0);
			double dphi1=tmp_dof*contacts[2*l].joint_ratio; // psi1 - phi1
			double dphi2=tmp_dof*contacts[2*l+1].joint_ratio; // psi2 - phi2

			double dphi1_cont=contacts[2*l].psi_cont*contacts[2*l].joint_ratio; // psi1_cont - phi1_cont
			double dphi2_cont=contacts[2*l+1].psi_cont*contacts[2*l+1].joint_ratio; // psi2_cont - phi2_cont

			double actuation_tau1=Tr1*(dphi1); // eq 1
			double actuation_tau2=Tr2*(dphi2); // eq 1

			fn2=contacts[2*l+1].world_normal;
			fn2*= (actuation_tau2 - k2*dphi2_cont )/contacts[2*l+1].a ; // eq 2
		
			ft2=contacts[2*l+1].world_tangent;
			ft2*= contacts[2*l+1].a*ks*( 1 - sqrt(1-fn2.len_sq()/pow(contacts[2*l+1].a*ks,2))); // eqn 3,4

			fn1=contacts[2*l].world_normal;
			vec3 tmp;
			tmp=(fn2*contacts[2*l].world_normal);
			double term1=tmp.len();
			tmp=(ft2*contacts[2*l].world_tangent);
			double term2=tmp.len();
			double force_mag=actuation_tau1 - abs(term1) - abs(term2) - abs(k1*dphi1_cont+phi1)/contacts[2*l].a;

			cerr << "a2+lcos : " << (contacts[2*l+1].a+ linkl*cos( dphi2_cont+phi2))/1000<<"\n";
			cerr << "lsin : "<<dphi2_cont+phi2<<" = " << linkl*sin( dphi2_cont+phi2)/1000<<"\n";
			cerr << "k : "<<k1*(dphi1_cont)<<"\n";
			cerr << "act tau : "<<actuation_tau1<<"\n";

			double force_mag1=
					actuation_tau1 
						- abs(0.001*fn2.len()*(contacts[2*l+1].a+ linkl*cos( dphi2_cont+phi2)))
						- abs(0.001*ft2.len()*linkl*sin( dphi2_cont+phi2))
                        - abs(1.0*k1*dphi1_cont)/contacts[2*l].a; // eqn 9
			if(force_mag != force_mag1)	{
				cerr << "force mag : "<<force_mag<<"	force mag1 : "<<force_mag1<<"\n";
					
			}
			if(force_mag < 0 ) force_mag=0;
			fn1*=force_mag;//(actuation_tau1 - term1 - term2 - k1*(contacts[2*l].joint_val*contacts[2*l].joint_ratio))/contacts[2*l].a; //eqn 9

			ft1=contacts[2*l].world_tangent;
			ft1*=contacts[2*l].a*ks*( 1 - sqrt(1-fn1.len_sq()/pow(contacts[2*l].a*ks,2))); // eqn 3,4

			u_force+=fn1.len()*contacts[2*l].body_normal;
			u_force+=fn2.len()*contacts[2*l+1].body_normal;
			u_force+=ft1.len()*contacts[2*l].body_tangent;
			u_force+=ft2.len()*contacts[2*l+1].body_tangent;

			cerr << "fn1 : "<<fn1.len()<<"\n";
			cerr << "ft1 : "<<ft1.len()<<"\n";
			cerr << "fn2 : "<<fn2.len()<<"\n";
			cerr << "ft2 : "<<ft2.len()<<"\n";
			
			vector< vec3 > bf_pair(2,vec3(0,0,0) );
			bf_pair[0]=fn1.len()*contacts[2*l].body_normal;
			bf_pair[0]+=ft1.len()*contacts[2*l].body_tangent;
			bf_pair[1]=fn1.len()*contacts[2*l+1].body_normal;
			bf_pair[1]+=ft1.len()*contacts[2*l+1].body_tangent;

			bforces.push_back(bf_pair);
		} else if(contacts[2*l].init && contacts[2*l].a>0 ) {
			// contact on link1 only 

			vec3 fn1(0,0,0);
			vec3 ft1(0,0,0);

			double dphi1=rad2deg(tmp_dof*contacts[2*l].joint_ratio); // psi1 - phi1

			double dphi1_cont=rad2deg(contacts[2*l].psi_cont*contacts[2*l].joint_ratio); // psi1_cont - phi1_cont

			double actuation_tau1=Tr1*(dphi1); // eq 1

			fn1=contacts[2*l].world_normal;
			fn1*= (actuation_tau1 - k2*dphi1_cont )/contacts[2*l].a ; // eq 2
		
			ft1=contacts[2*l].world_tangent;
			ft1*= contacts[2*l].a*ks*( 1 - sqrt(1-fn1.len_sq()/pow(contacts[2*l].a*ks,2))); // eqn 3,4

			u_force+=fn1.len()*contacts[2*l].body_normal;
			u_force+=ft1.len()*contacts[2*l].body_tangent;

			cerr << "fn1 : "<<fn1.len()<<"\n";
			cerr << "ft1 : "<<ft1.len()<<"\n";

			vector< vec3 > bf_pair(2,vec3(0,0,0) );
			bf_pair[0]=fn1.len()*contacts[2*l].body_normal;
			bf_pair[0]+=ft1.len()*contacts[2*l].body_tangent;
			
			bforces.push_back(bf_pair);
		} else if( contacts[2*l+1].init && contacts[2*l+1].a>0 ) {
			// contact on link 2 only
			
			vec3 fn2(0,0,0);
			vec3 ft2(0,0,0);
			
			double dphi2=rad2deg(tmp_dof*contacts[2*l+1].joint_ratio); // psi2 - phi2
			
			double dphi2_cont=rad2deg(contacts[2*l+1].psi_cont*contacts[2*l+1].joint_ratio); // psi2_cont - phi2_cont

			
			double actuation_tau2=Tr2*(dphi2); // eq 1


			fn2=contacts[2*l+1].world_normal;
			fn2*= (actuation_tau2 - k2*dphi2_cont )/contacts[2*l+1].a ; // eq 2
			
	
			ft2=contacts[2*l+1].world_tangent;
			ft2*= contacts[2*l+1].a*ks*( 1 - sqrt(1-fn2.len_sq()/pow(contacts[2*l+1].a*ks,2))); // eqn 3,4


			u_force+=fn2.len()*contacts[2*l+1].body_normal;
			u_force+=ft2.len()*contacts[2*l+1].body_tangent;
						
			cerr << "fn2 : "<<fn2.len()<<"\n";
			cerr << "ft2 : "<<ft2.len()<<"\n";

			vector< vec3 > bf_pair(2,vec3(0,0,0) );
			
			bf_pair[1]=fn2.len()*contacts[2*l+1].body_normal;
			bf_pair[1]+=ft2.len()*contacts[2*l+1].body_tangent;

			bforces.push_back(bf_pair);
	
		}
		//else no contact, skip this finger
		
		
	}


	cerr << "returning " << u_force.len() <<"\n";
					
	return u_force.len();
}
											 

void HarvardHand::autoGrasp(bool renderIt,double speedFactor)
{

	double retval=0;
	double u_force;
	double last_dof=0;
	int i,j,f,l,d;
	bool done;
	ColReportT result;

	for(i=0;i<8;i++)
	{
		contacts[i].init=0;
	}



	bool   *moving = new bool[numDOF];
	double *currVals = new double[numDOF];
	double *newVals = new double[numDOF];
	double *desiredVals = new double[numDOF];
	double *desiredSteps= new double[numDOF];
	double *stepSize= new double[numDOF];
	for (i=0;i<numDOF;i++) 
	{
		desiredVals[i] = dofVec[i]->getMax();
		desiredSteps[i] = dofVec[i]->getDefaultVelocity()*speedFactor*AUTO_GRASP_TIME_STEP;
	}


	for (i=0;i<numDOF;i++) {
		if (!desiredSteps || desiredSteps[i] == WorldElement::ONE_STEP )
			stepSize[i] = desiredVals[i] - dofVec[i]->getVal();
		else
			stepSize[i] = desiredSteps[i];
		moving[i] = true;
		if (stepSize[i] == 0.0 || desiredVals[i] == dofVec[i]->getVal() )
			moving[i] = false;
		if ( moving[i] ) done = false;
	}

	for (f=0; f<getNumChains(); f++) {
		for (l=0; l<getChain(f)->getNumLinks(); l++ ) {
			getChain(f)->getLink(l)->setMoving(true);
		}
	}

	int itercount = 0;
	do {
		getDOFVals(currVals);
		for (d=0;d<numDOF;d++) {
			newVals[d] = currVals[d] + stepSize[d];
			if (stepSize[d] > 0 && newVals[d] > desiredVals[d])
				newVals[d] = desiredVals[d];
			else if (stepSize[d] < 0 && newVals[d] < desiredVals[d])
				newVals[d] = desiredVals[d];
		}

		result = forceDOFTo(newVals,1);

		if (result.size() != 0)
		{
			
			// we have a contact
			for (unsigned int i=0;i < result.size();i++) 
			{
				// calculate forces due to previous contacts at this dof
				// then add each contact to the list of contacts 
				// at the moment of contact, the force applied is zero


				if (result[i].first->inherits("Link") && ((Link *)result[i].first)->getOwner() == this) 
				{

					f = ((Link *)result[i].first)->getChainNum();
					l = ((Link *)result[i].first)->getLinkNum();

					// calculate force at this dof for all previous contacts
					if(i==0)
					{
						for(unsigned int j=0;j<chainVec.size();j++)
						{
							double d1=(chainVec[j])->getJoint(1)->getVal();
							double d0=(chainVec[j])->getJoint(0)->getVal();
							if(j==0 || last_dof < d0 )
							{
								last_dof=d0;
							}
							if(last_dof < d1)
								last_dof=d1;
						}
						
						u_force=current_unbalanced_force(last_dof);
					}

					vec3 world_normal=((Link *)result[i].first)->getContacts().front()->getNormal() * ((Link *)result[i].first)->getContacts().front()->getBody1Tran() ;
					vec3 body_normal=world_normal * ((Link *)result[i].first)->getContacts().front()->getBody2Tran().inverse();


					// add this contact to the list of contacts
					if(!contacts[f*2+l].init)
						contacts[f*2 + l].set_vals(
						1,
						body_normal,			 // normal wrt body
						body_normal *(0,0,1), //tangent wrt body
						world_normal, // normal wrt world
						world_normal*(0,0,1) , // tangent wrt world
						((Link *)result[i].first)->getContacts().front()->getPosition().toSbVec3f().length(),
						(chainVec[f])->getJoint(l)->getRatio(),
						(chainVec[f])->getJoint(l)->getVal()
						);

				}
				else if (result[i].second->inherits("Link") && ((Link *)result[i].second)->getOwner() == this) 
				{
					f = ((Link *)result[i].second)->getChainNum();
					l = ((Link *)result[i].second)->getLinkNum();
					if(i==0)
					{
						for(unsigned int j=0;j<chainVec.size();j++)
						{
							double d1=(chainVec[j])->getJoint(1)->getVal();
							double d0=(chainVec[j])->getJoint(0)->getVal();
							if(j==0 || last_dof < d0 )
							{
								last_dof=d0;
							}
							if(last_dof < d1)
								last_dof=d1;
						}
						

						u_force=current_unbalanced_force(last_dof);
					}


					vec3 world_normal=((Link *)result[i].second)->getContacts().front()->getNormal() * ((Link *)result[i].second)->getContacts().front()->getBody1Tran() ;
					vec3 body_normal=world_normal * ((Link *)result[i].second)->getContacts().front()->getBody2Tran().inverse();

					if(!contacts[f*2+l].init)
					contacts[f*2 + l].set_vals(
						1,
						body_normal,
						body_normal *(0,0,1),
						world_normal,
						world_normal *(0,0,1),
						((Link *)result[i].second)->getContacts().front()->getPosition().toSbVec3f().length(),
						(chainVec[f])->getJoint(l)->getRatio(),
						(chainVec[f])->getJoint(l)->getVal()
						);


				}
				if(u_force > retval)
				{
					retval=u_force;
					cerr << "Changing max unbalanced force to : "<<retval<<"\n";
				}
			}
		}

		itercount++;

		//stop DOF's that have reached the target
		for (i=0;i<numDOF;i++) 
		{
			if (dofVec[i]->getVal() == desiredVals[i]) moving[i] = false;
		}

		//stop DOF's of links that have hit something
		for (f=0; f<getNumChains(); f++) {
			for (l=0; l<getChain(f)->getNumLinks(); l++ ) {
				if ( !getChain(f)->getLink(l)->isMoving() ) {
					for (j=getChain(f)->getLastJoint(l); j>=0; j--) {
						d = getChain(f)->getJoint(j)->getDOFNum();
						desiredVals[d] = dofVec[d]->getVal();
						stepSize[d] = 0.0;
						moving[d] = false;
					}
				}
			}
		}
		done = true;
		for (i=0;i<numDOF;i++)
			if (moving[i]) {done=false;}
	} while (!done);

	u_force=current_unbalanced_force(last_dof);
	if(u_force > retval)
	{
		retval=u_force;
		cerr << "Changing max unbalanced force to : "<<retval<<"\n";
	}
	

	for(unsigned int i=0;i<8 ; i++ )
	{
		cerr << " link : " << i 
			<< "\n  a : "<< contacts[i].a
			<< "\n  dof : "<< contacts[i].joint_val
			<< "\n  joint ratio : "<< contacts[i].joint_ratio
			<< "\n  psi cont : "<< contacts[i].psi_cont
			<< "\n  bnx : "<< contacts[i].body_normal.x()
			<< "\n  bny : "<< contacts[i].body_normal.y()
			<< "\n  bnz : "<< contacts[i].body_normal.z()
			<< "\n  btx : "<< contacts[i].body_tangent.x()
			<< "\n  bty : "<< contacts[i].body_tangent.y()
			<< "\n  btz : "<< contacts[i].body_tangent.z()
			<< "\n  wnx : "<< contacts[i].world_normal.x()
			<< "\n  wny : "<< contacts[i].world_normal.y()
			<< "\n  wnz : "<< contacts[i].world_normal.z()
			<< "\n  wtx : "<< contacts[i].world_tangent.x()
			<< "\n  wty : "<< contacts[i].world_tangent.y()
			<< "\n  wtz : "<< contacts[i].world_tangent.z()
			<< "Max unbalanced force : "<<retval<<"\n";
	}

	delete [] currVals;
	delete [] newVals;
	delete [] moving;
	delete [] desiredVals;
	delete [] stepSize;
	return ;



}



/*
void
HarvardHand::autoGrasp(bool renderIt,double speedFactor)
{
  int i,f,l,numCols;
  bool done=false;
  double *desiredVals = new double[numDOF];
  
  ColReportT colReport;

#ifdef GRASPITDBG
  std::cout << "in autograsp" << std::endl;
#endif

  if (myWorld->dynamicsAreOn()) {
    for (i=0;i<numDOF;i++) {
      if (dofVec[i]->getDefaultVelocity() > 0)
	desiredVals[i] = dofVec[i]->getMax();
      else if (dofVec[i]->getDefaultVelocity() < 0)
	desiredVals[i] = dofVec[i]->getMin();
      else desiredVals[i] = 0.0;
#ifdef GRASPITDBG
      std::cout <<"Desired val "<<i<<" "<<desiredVals[i]<<std::endl;
#endif
      //for now
      dofVec[i]->setDesiredVelocity(dofVec[i]->getDefaultVelocity());
    }
    setDesiredDOFVals(desiredVals);
	delete [] desiredVals;
    return;
  }

  bool *moving = new bool[numDOF];
  double *stepSize= new double[numDOF];
  double *origVals = new double[numDOF];

  for (i=0;i<numDOF;i++) {
    origVals[i] = dofVec[i]->getVal();
    desiredVals[i] = dofVec[i]->getMax();
    stepSize[i] = dofVec[i]->getDefaultVelocity()*speedFactor*
      AUTO_GRASP_TIME_STEP;
    moving[i] = (stepSize[i] != 0.0);
  }

  while (!done) {
#ifdef GRASPITDBG
    for (i=0;i<numDOF;i++)
      if (moving[i]) std::cout << i << " is moving" << std::endl;
      else std::cout << i << " is not moving" <<std::endl;

    std::cout << "calling movedof" << std::endl;
#endif
    colReport = moveDOFTo(desiredVals,stepSize,renderIt);
    for (i=0;i<numDOF;i++) {
      if (dofVec[i]->getVal() == desiredVals[i]) moving[i] = false;
    }
      
    numCols = colReport.size();

    for (i=0;i<numCols;i++) {
      if (colReport[i].first->inherits("Link") &&
	  ((Link *)colReport[i].first)->getOwner() == this) {	
	f = ((Link *)colReport[i].first)->getChainNum();
	l = ((Link *)colReport[i].first)->getLinkNum();
	if (f == 0 && l == 2) {
	  desiredVals[1] = dofVec[1]->getVal();
	  moving[1] = false;
	} else if (f==1 && l==2) {
	  desiredVals[2] = dofVec[2]->getVal();
	  moving[2] = false;
	} else if (f==2 && l==1) {
	  desiredVals[3] = dofVec[3]->getVal();
	  moving[3] = false;
	}
      }
      if (colReport[i].second->inherits("Link") &&
	  ((Link *)colReport[i].second)->getOwner() == this) {
	f = ((Link *)colReport[i].second)->getChainNum();
	l = ((Link *)colReport[i].second)->getLinkNum();
	if (f==0 && l==2) {
	  desiredVals[1] = dofVec[1]->getVal();
	  moving[1] = false;
	} else if (f==1 && l==2) {
	  desiredVals[2] = dofVec[2]->getVal();
	  moving[2] = false;
	} else if (f==2 && l==1) {
	  desiredVals[3] = dofVec[3]->getVal();
	  moving[3] = false;
	}
      }
    }

    done = true;
    for (i=0;i<numDOF;i++)
      if (moving[i]) {done=false;}
  }

  delete [] origVals;
  delete [] desiredVals;
  delete [] moving;
  delete [] stepSize;
}
*/

QTextStream&
HarvardHand::readDOFVals(QTextStream &is)
{
  double dofVals[4];
  double reconnectAt;
  int i;

  std::cout << "IN HarvardHand READ"<<std::endl;
  for (i=0;i<4;i++)
    is >> dofVals[i];

  for (i=1;i<4;i++) {
    is >> reconnectAt;
    if (reconnectAt < dofVals[i]) {
      setDOFVal(i,reconnectAt);
      ((HarvardHandDOF *)dofVec[i])->breakAway();
    }
  }
  
  setDOFVals(dofVals);
  return is;
}

QTextStream&
HarvardHand::writeDOFVals(QTextStream &os)
{
  double dofVals[4];
  int i;

  os << myFilename << endl;
  getDOFVals(dofVals);

  for (i=0;i<4;i++)
    os << ' ' << dofVals[i]; 

  for (i=1;i<4;i++)
    if ( ((HarvardHandDOF *)dofVec[i])->inBreakAway() )
      os << ' ' << ((HarvardHandDOF *)dofVec[i])->getReconnectAt();
    else
      os << ' ' << dofVals[i];

  os << endl;

  return os;
}


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
//######################################################################

/*! \file 
  \brief Defines the %HarvardHand hand class, a specialized hand subclass
 */
#ifndef HARVARDHAND__H
#define HARVARDHAND__H

#include "joint.h"
#include "robot.h"
#include <vector>
#include <QTextStream>

class HarvardHand;

//! The HarvardHand hand has a special DOF that allows the passively coupled joint to breakaway
/*! The HarvardHand hand has a special feature that allows the passively coupled
    outer joint of each finger to continue to close if the inner link is
    stopped by a contact.  As the finger opens again the joint becomes
    recoupled when it reaches the point when the breakaway first occurred.
    This only works when dynamics is off.  We just need to add some code to
    check a torque threshold to determine when breakaway occurs during
    dynamic simulation.
*/

struct harvardContactStruct
{
	//dof at contact
	double joint_val;

	// is this structure initialized ?
	bool init;

	// contact distance 
	double a;

	// angle at contact
	double psi_cont;

	// dof * joint_ratio = apparent angle
	double joint_ratio;

	// normal wrt world coords
	vec3 world_normal;

	// tangent wrt body coords
	vec3 world_tangent;

	// normal wrt body coords
	vec3 body_normal;

	// tangent wrt body coords
	vec3 body_tangent;

	harvardContactStruct()
	{
		init=0;
		a=0;
	}
	void set_vals(bool tinit,
			vec3 bn,
			vec3 bt,
			vec3 wn,
			vec3 wt,
			double ta, 
			double jr,
			double psi)
	{
		init=tinit;
		body_normal=bn;
		body_tangent=bt;
		world_normal=wn;
		world_tangent=wt;
		a=ta;
		joint_ratio=jr;
		psi_cont=psi;
	}
};

class HarvardHandDOF : public DOF {
  friend class HarvardHand;

  //! Holds the original joint list while this DOF is in breakaway
  std::list<Joint *> tempList;
  

  //! Keeps track of the DOF values at which the breakaway occurred
  double reconnectAt;
  
  //! Keeps track of the DOF values at which the breakaway occurred
  //! 1 entry per joint that this dof is connected to
  std::vector< std::vector<double> > reconnectAts;


  /*! Checks the new DOF value.  If it is less than the reconnectAt and the
      DOF is in breakaway, it reconnects the inner joint to the DOF. */
  int setVal(double q1);

 public:
	
  /*! Copies the data from the generic DOF into the new HarvardHand DOF */
  HarvardHandDOF(const DOF &oldDOF);

  /*! Empty destructor (placeholder)*/
  virtual ~HarvardHandDOF() {}

  /*! Returns whether the DOF is in breakaway mode or not. */
  bool inBreakAway() const {return !tempList.empty();}

  /*! Returns the value at which the inner joint will reconnect to the DOF */
  double getReconnectAt() const {return reconnectAt;}

  /*! Saves the current joint list to templist and removes the first joint
      from the original list, thus disconnecting it from the DOF.
  */
  void breakAway();

  /*! Restores the original joint list, thus reconnecting the inner joint to
      the DOF.
  */
  void reconnect();
};

//! A special hand with a clutch mechanism in each finger that allows the outer joint to continue to close after the inner link is stopped.
class HarvardHand : public Hand {
protected:

  /*! Calls the moveDOFTo in the robot class, but if any of the inner links
      are stopped by a contact, this will call the breakaway routine of the
      associate HarvardHandDOF.
  */
  ColReportT forceDOFTo(double *desiredDOFVals);

  bool breakAway(int f, int l, int d);

 public:
  /*! Empty constructor (placeholder) */
  HarvardHand(World *w,const char *name) : Hand(w,name) {}

  /*! Performs the usual robot load, but replaces 3 of the DOF's with 
    HarvardHandDOF's */
  int load(QString filename);

  bool linkContactsPreventDOFMotion(int c, int l, const transf& motion);

  ColReportT forceDOFTo(double *desiredDOFVals, bool fastCollision = false);

  /*! This overrides the original fwdKinematics so that the correct
      values are used when one or more fingers have broken away. */
  void fwdKinematics(double* dofVals,std::vector<transf>& trVec,int chainNum);

  //  void autoGrasp(bool renderIt,double speedFactor);  // not used right now

  /*! returns the max unbalanced force when this robot grasps the object */
  //double HarvardHand::maxUnbalancedForce(double speedFactor);
  void autoGrasp(bool renderIt,double speedFactor=1.0);

  /*! helper function to convert angles from radians to degrees */
	inline static double rad2deg(double angle)
	{
		return 57.2957795*angle;
	}
	inline static double deg2rad(double angle)
	{
		return angle/57.2957795;
	}
  /*! Since the hand posture may be saved while one or more joints have broken
      away, it must read in ALL joint values.
  */
  QTextStream& readDOFVals(QTextStream &is);

  /*! Since the hand posture may be saved while one or more joints have broken
      away, it must write out ALL joint values.
  */
  QTextStream& writeDOFVals(QTextStream &os);

  private:
	  /*! to store contacts for calculating max unbalanced force */
	  harvardContactStruct contacts[8];
	  double HarvardHand::current_unbalanced_force(double tmp_dof);

	  // define a vector to store all the intermediate forces
	  // that we calculate. This vector has maximum of 8 elements (corresponding
	  // to the 8 times contacts can be made. Every element of bforces
	  // (bforces stands for body-forces) must have 2 forces, fn1 and fn2
	  // (refer to writeup). Each of fn1 and fn2 is a vec3. We will be 
	  // pushing new values into bforces with the push_back routine.
	  std::vector< std::vector< vec3 > > bforces;

};


#endif

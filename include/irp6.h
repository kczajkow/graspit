#ifndef IRP6_H
#include "robot.h"

//! Specifically for the IRp6 Arm.  Contains analytical inverse kinematics solution
/*! This class was created specifically for the IRp6 Arm because we can
    override the generic inverse kinematics routine with one that solves them
    analytically.  
*/
class IRp6 : public Robot {

 public:
  /* Empty constructor */
  IRp6(World *w,const char *name) : Robot(w,name) {}

  /* Solves the inverse kinematics of the IRp6 arm analytically. */
  virtual int invKinematics(const transf& endTran,double* dofVals,int chainNum=0);
};

#define IRP6_H
#endif
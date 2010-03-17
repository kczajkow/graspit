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
// $Id: grasp.h,v 1.18.2.2 2009/04/27 14:33:09 cmatei Exp $
//
//######################################################################

/*! \file 
  \brief Defines the grasp class, which analyzes grasps
 */
#ifndef GRASP_HXX

#include <list>
#include <vector>
#include <set>
#include <QObject>

//Inventor includes
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
class SoTransform;
class SoCoordinate3;
class SoIndexedFaceSet;

class vec3;
class transf;
class position;
class Hand;
class GraspableBody;
class Contact;
class GWS;
class QualityMeasure;
class QMDlg;
class GWSprojection;
class Matrix;

//! Max iteration steps in a feasibility phase
#define MAX_FEAS_LOOPS	100 
//! Max iteration steps in a optimization phase
#define MAX_OPTM_LOOPS  100 

extern bool saveSetup;
extern int saveCounter;

//! A grasp occurs between a hand and an object and has quality measures associated with it.  
//  It also has methods to optimize grasping forces.
/*! Each hand object has a grasp associated with it, and the grasp occurs
    between the hand and a graspableBody.  Each grasp is incharge of
	maintaining	a set of computed grasp wrench spaces, which are then used
	by various quality measures.  When contacts change between the two, the 
	grasp is updated, meaning each grasp wrench space associated with the 
	grasp is rebuilt, as well as any grasp wrench space projections.  All
    grasp quality measures are also re-evaluated. 

	The grasp wrench spaces are designed to be shared by different quality 
	metrics. When a new metric is added, if a GWS of the needed type already
	exists in the list maintained by thhis grasp, the new QM will use it, 
	and a reference count on the GWS is increased. GWS are automatically 
	deleted when no QM that needs them exist anymore.
	
	A grasp can also perform many of the same services on set of 
	VirtualCotacts (see documentation of that class for details). This part
	however has not neen competely tested, and might produce behavior that
	is not entirely consistent with how it handles traditional contacts.

	The grasp class also contains several methods that are used for grasp 
	force optimization. These use a linear matrix inequality technique and 
	are adapted from code written by Li Han and Jeff Trinkle.  More
	information regarding their algorithm can be found in: L. Han, 
	J. Trinkle, and Z. Li, "Grasp Analysis as Linear Matrix Inequality 
	Problems," \e IEEE \e Transactions \e on \e Robotics \e and \e 
	Automation, Vol. 16, No. 6, pp. 663--674, December, 2000. Note that
	this functionality is complete, but it has not been throroughly used
	or tested. Is is also not completely documented, so you will have to read
	the code and figure it out. We will refer to code that belongs to it 
	as GFO (Grasp Force Optimization) routines.

	Recently, we have added a new type of quasi-static force optimization 
	routines geared	towards underactuated hands. The grasp can compute if 
	its contacts are stable or they will slip on the object, and also the 
	unblanced force applied on its object due to the underactuated 
	mechanism. Most of these computations are performed in the 
	KinematicChain class, but the Grasp also uses them.
*/
class Grasp : public QObject{
  Q_OBJECT

  //! A pointer to the hand that owns this grasp object
  Hand *hand;

  //! A pointer to the object that is the focus of this grasp
  GraspableBody *object;

  //! \c TRUE if the grasp has been updated since the last time the contacts changed
  bool valid;

  //! A list of pointers to the associated grasp wrench spaces
  std::list<GWS *> gwsList;

  //! A list of pointers to the associated quality measures
  std::list<QualityMeasure *> qmList;

  //! A list of pointer to the associated grasp wrench space projections
  std::list<GWSprojection *> projectionList;

  //! Number of quality meausre in the list
  int numQM;

  //! A vector of pointers to the contacts on the object where it touches the hand 
  std::vector<Contact *> contactVec;

  //! Number of grasp contacts
  int numContacts;

  //! Used when saving output from grasp force optimization analysis
  int graspCounter;

  //! Minimum grasp wrench that can be applied given contact forces that sum to 1
  double minWrench[6];  

  //! Tells us if quality metrics should take into account gravity
  bool useGravity;

  //! Computes the Jacobian of a link wrt the base of the finger, in link coordinates
  double *getLinkJacobian(int f, int l);

  //! Sets the reference point that is used for grasp wrench computations as the center of the virtual contacts
  void setVirtualCentroid();

  //! Computes the virtual center of the internally assembled list of virtual contacts
  vec3 virtualCentroid();

  //! Sets the reference points of all virtual contacts using the c.o.g of the given object 
  void setRealCentroid(GraspableBody *body);

  /***** Additions from Li Han's code to do grasp force optimization *********/
  //! Number of basis wrenches for grasp map (3 * numContacts for PCWF)
  int numWrenches; 

  //! A \a (6 * numWrenches) matrix containing basis wrenches for each contact vertex (stored in column-major format)
  double *graspMap; 

  //! Dimension of the null space of the grasp map
  int nullDim;

  //! The null space of the grasp map; (column-major)
  double *nullSpace;

  //! vector of size \a (numWrenches+1), the optimal grasp contact forces, computed from optmz0 and the objective value.
  double *optmx0;   

  //! vector of size \a numDOF storing the optimal torque for each DOF
  double *optTorques; 

  //! Min norm solution to \f$\mbox{GraspMap} * x = F_{ext}\f$
  double *minNormSln;

  //! The number of degrees of freedom of the hand
  int numDOF;

  //! The grasp jacobians
  double *Jacobian;

  //! temporary: should come from hand object
  double *externalTorques; 
  
  //! These variables are used by maxdet package.  Refer to maxdet manual for their explainations.
  int L, K, GPkRow, *F_blkszs,*G_blkszs;                 

  //! These variables are used by maxdet package.  Refer to maxdet manual for their explainations.
  double *F, *G, *Z, *W, *c, constOffset;

  //! Termination criteria used in maxdet algorithm. Refer to maxdet manual.
  double gamma, abstol, reltol; 

  //! If it is 1, then terminate the force feasibility phase whenever the objective value becomes negative, i.e. find one valid grasp force. Otherwise, use the regular maxdet termination.
  int negativeFlag;

  //! Is the grasp force optimization feasible?
  int feasible;

  //! On entry, the maximum number of total Newton iteraions, and on exit, the real number of Newton Iterations in the feasible phase.
  int feasNTiters;

  //! Vector of size \a m, the \a z value corrsponding to the initial feasible grasp forces computed in force feasibility phase.
  double *initz0;

  //! The counterpart to feasNTiters
  int optmNTiters;

  //! Vector of size \a m, the \a z value corresponding to optimal grasp forces, computed in force optimization phase.
  double *optmz0;

  //! Vector of size \a (m+3), \a optmz0, as well as the corresponding objective value, duality gap, and number of optimization iteration steps.
  double *extendOptmz0;

  //! array of dimension \a (m+3)*Number_of_Iterations_at_the_Feasibility_Phase, the history (iterative values) of z, objective value, duality gap and iteration number in force feasibility phase at each simulation step.
  double *feasZHistory; 

 // array of dimension \a (m+3)*Number_of_Iterations_at_the_Optimization_Phase, the counterpart to feasZHistory in the optimization phase.
  double *optmZHistory;

  //! array of dimension \a (m+3)*(Number_of_Iterations_at_Optimization_Phase+1), optmZHistory, + 1: the initial feasible grasp force, its objective value, duality gap and number of iterations 
  double *extendOptmZHistory; 

 //! array of dimension \a (NumWrenches+1)*Number_of_Iterations_at_the_Feasibility_Phase, the history of grasp forces x and objective value in the force feasibility phase at the last simulation step.
  double *feasXHistory;

 //! array of dimension \a (NumWrenches+1)*(Number_of_Iterations_at_the_Optimization_Phase+1), the counterpart to feasXHistory in the optimization phase at the last step.
  double *optmXHistory;

  //! Output file pointer for saving the results of a GFO analysis
  FILE *pRstFile;

  //! GFO routine
  void minimalNorm();
  //! GFO routine
  void computeNullSpace();
  

  //! Friction LMI helper function
  void lmiFL(double *lmi,int rowInit, int colInit, int totalRow);
  //! Friction LMI helper function
  void lmiPCWF(double cof, double *lmi,int rowInit, int colInit, int totalRow);
  //! Friction LMI helper function
  void lmiSFCE(double cof, double cof_t,double *lmi,int rowInit, int colInit,
	       int totalRow);
  //! Friction LMI helper function
  void lmiSFCL(double cof, double cof_t,double *lmi,int rowInit, int colInit,
	       int totalRow);
  //! Friction LMI helper function
  double *lmiTorqueLimits();
  //! Friction LMI helper function
  void lmiFL();
  //! Friction LMI helper function
  void lmiPWCF();
  //! Friction LMI helper function
  void lmiSFCE();
  //! Friction LMI helper function
  void lmiSFCL();
  //! Friction LMI helper function
  double *lmiFrictionCones();

  //! GFO routine
  double *weightVec();
  //! GFO routine
  void feasibilityAnalysis();
  //! GFO routine
  void optm_EffortBarrier();
  //! GFO routine
  void computeObjectives();
  //! GFO routine
  double *xzHistoryTransfrom(double *zHistory,int numIters);

  /**************************************************************************/
  friend class QMDlg;

signals:
  //! Called when contacts have changes and the wrench spaces need to be updated
  void graspUpdated();

public:
  Grasp(Hand *h);

  ~Grasp();

  /*! Returns whether the grasp has been updated since the last time grasp
    contacts have changed. */
  bool                    isValid() const {return valid;}

  /*! Returns whether the grasp force optimization problem is feasible. */
  bool                    isFeasible() const {return feasible;}

  /*! Returns the number of quality measures defined for this grasp. */
  int                     getNumQM() const {return numQM;}

  /*! Return a pointer to the object that is the focus of this grasp. */
  GraspableBody *         getObject() const {return object;}

  /*! Return the number of grasp contacts. */
  int                     getNumContacts() const {return numContacts;}

  /*! Return a pointer to the i-th grasp contact on the object. */
  Contact *               getContact(int i) const {return contactVec[i];}

  /*! Return a pointer to the grasp jacobian matrix. */
  double *                getJacobian() const {return Jacobian;}

  /*! Return the optimal DOF forces (the results of GFO). */
  double *                getOptDOFEfforts() const {return optTorques;}

  void                    getMinWrench(double *w) const
  {
    if (w) memcpy(w,minWrench,6*sizeof(double));
  }

  void                    setMinWrench(double *w)
  {
    if (w) memcpy(minWrench,w,6*sizeof(double));
  }

  /*! Sets graspableBody \a g to be the new focus of the grasp and updates the
    grasp. */
  void                    setObject(GraspableBody *g) {object = g; update();}

  /* this is a hack; I had to do it due to some bug I was never able to trace down*/
  void                    setObjectNoUpdate(GraspableBody *g) {object = g;}

  //! Collects all the contacts between the hand and the object in an internal list
  void collectContacts();
  //! Collects all the virtual contacts on the hand n an internal list
  void collectVirtualContacts();

  //! Builds the grasp map computing net object wrench from contact wrenches
  void buildGraspMap();
  //! Constructs a version of the hand jacobian 
  void buildJacobian();
  //! An old version for computing contact force limits based on actuator limits
  void computeContactForceLimits();

  //! Collects all the contacts in the internal list and updates the wrench spaces
  void update();
  //! Updates (re-computes) the wrench spaces of this grasp and all of their projections
  void updateWrenchSpaces();
  //! A simpler version that only build 3D GWS (instead of the usual 6D). Not recommended for use.
  void updateWrenchSpaces3D();

  //! Returns the max radius used in GWS computations, either from the object of from virtual contacts
  double getMaxRadius();
  //! Returns the c.o.g. used in GWS computations, either from the object of from virtual contacts
  position getCoG();

  //! Adds a GWS of a given type to the grasp, unless one exists already
  GWS *addGWS(const char *type);
  //! Decrements the reference count on a GWS of the given type, and deletes it if the ref count reaches 0
  void removeGWS(GWS *gws);
  //! Adds a quality measure to this grasp
  void addQM(QualityMeasure *qm);
  //! Replaces a QM in the list associated with this grasp with another QM
  void replaceQM(int which,QualityMeasure *qm);
  //! Returns one of the quality measures that have been associated with this grasp
  QualityMeasure *getQM(int which);
  //! Removes a quality measure that has been associated with this grasp
  void removeQM(int which);
  
  void addProjection(GWSprojection *gp);
  void removeProjection(GWSprojection *gp);
  static void destroyProjection(void * user, SoQtComponent * component);

  //! Sets whether QM's should take gravity into account; not very thoroughly tested yet
  void setGravity(bool g){useGravity = g;}
  bool isGravitySet(){return useGravity;}

  //! GFO parameter
  static double GFO_WEIGHT_FACTOR;
  //! Main GFO routine
  int  findOptimalGraspForce();

  //! A different version of computing the contact Jacobian, used for quasi-static analysis
  Matrix contactJacobian(bool worldCoords);
  //! Main quasi-static analysis routine for underactuated hands
  int computeQuasistaticForces(bool useDynamicDofForce);
};

#define GRASP_HXX
#endif

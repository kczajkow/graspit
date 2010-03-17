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
// Author(s): Matei T. Ciocarlie
//
// $Id: mosek_qp.cpp,v 1.6.2.2 2009/04/27 14:33:16 cmatei Exp $
//
//######################################################################

/*! \file
	A wrapper for using the Mosek commercial QP solver from within GraspIt!
*/

#include "mosek_qp.h"

#include <math.h>

#include "matrix.h"
//#define GRASPITDBG
#include "debug.h"

#include "mosek.h"

static void MSKAPI printstr(void*, char str[])
{
	DBGP(str);
}

/*! Creates a Mosek environment singleton and deletes it when the instance
	of this goes out of scope (presumably at the end of the program */
class MosekEnvAutoPtr {
public:
	MSKenv_t     env;
	MosekEnvAutoPtr() {
		MSKrescodee r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
		if ( r!=MSK_RES_OK ) {
			DBGA("Failed to create Mosek environment");
			assert(0);
		}
		MSK_linkfunctoenvstream(env,MSK_STREAM_LOG,NULL,printstr);
		r = MSK_initenv(env);
		if ( r!=MSK_RES_OK ) {
			DBGA("Failed to initialize Mosek environment");
			assert(0);
		}
	}
	~MosekEnvAutoPtr() {
		DBGA("Mosek environment cleaned up");
		MSK_deleteenv(&env);
	}
};

MSKenv_t& getMosekEnv(){
	//the one and only instance of the Mosek environment
	static MosekEnvAutoPtr mskEnvPtr;
	return mskEnvPtr.env;
}

/*! The first \a numNN variables are constrained to be non-negative. In numNN = -1
	all variables are constrained to be non-negative */
int mosekNNQPSolverWrapper(const Matrix &Q, const Matrix &Eq, const Matrix &b,
						   const Matrix &InEq, const Matrix &ib, Matrix &sol, 
						   double *objVal, int numNN)
{
	DBGP("Mosek QP Wrapper");
	MSKrescodee  r;
	MSKtask_t task = NULL; 

	// Get the only instance of the mosek environment.
	MSKenv_t     env  = getMosekEnv();
    // Create the optimization task.
    r = MSK_maketask(env, 0, 0, &task);
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to create optimization task");
		return -1;
	}
    MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);
	
	//---------------------------------------
	//start inputing the problem
	//prespecify number of variables to make inputting faster
	r = MSK_putmaxnumvar(task, sol.rows());
	//number of constraints (both equality and inequality)
	if (r == MSK_RES_OK) {
		r = MSK_putmaxnumcon(task, Eq.rows() + InEq.rows());
	}
	//number of non-zero entries in A
	//for now we assume both Eq and InEq are dense, although they might not be
	if (r == MSK_RES_OK) {
		r = MSK_putmaxnumanz(task, (Eq.rows() + InEq.rows())*Eq.cols() );
	}
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to input variables");
		MSK_deletetask(&task);
		return -1;
	}

	//---------------------------------------
	//insert the actual variables and constraints

	//append the variables
	MSK_append(task, MSK_ACC_VAR,sol.rows());
	//append the constraints. 
	MSK_append(task, MSK_ACC_CON,Eq.rows() + InEq.rows());

	//the quadratic optimization objective
	//free term is zero
	//MSK_putcfix(task, 0.0);
	//all linear terms are zero
	//for (int i=0; i<Q.cols(); i++) {
		//MSK_putcj(task, i, 0.0);
	//}
	//the quadratic term
	for (int i=0; i<Q.rows(); i++) {
		for (int j=0; j<Q.cols(); j++) {
			if ( fabs(Q.elem(i,j))>1.0e-5) {
				MSK_putqobjij(task, i, j, 2.0*Q.elem(i,j));
			}
		}
	}

	//variable bounds, the first numNN assumed to be non-negative
	if (numNN < 0) numNN = sol.rows();
	assert(numNN <= sol.rows());
	for (int i=0; i<numNN; i++) {
		MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_LO, 0.0, +MSK_INFINITY);
	}
	for (int i=numNN; i<sol.rows(); i++) {
		MSK_putbound(task, MSK_ACC_VAR, i, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
	}

	//constraints and constraint bounds
	for (int i=0; i<Eq.rows(); i++) {
		for (int j=0; j<Eq.cols(); j++) {
			if (Eq.elem(i,j)!=0.0) {
				MSK_putaij( task, i, j, Eq.elem(i,j) );
			}
		}
		//equality constraints
		MSK_putbound(task, MSK_ACC_CON, i, MSK_BK_FX, b.elem(i,0), b.elem(i,0));
	}
	for (int i=0; i<InEq.rows(); i++) {
		int eqi = i + Eq.rows();
		for (int j=0; j<InEq.cols(); j++) {
			if (InEq.elem(i,j)!=0.0) {
				MSK_putaij( task, eqi, j, InEq.elem(i,j) );
			}
		}
		//inequality constraints, <=
		MSK_putbound(task, MSK_ACC_CON, eqi, MSK_BK_UP, -MSK_INFINITY, ib.elem(i,0));
	}

	//specify objective: minimize
	MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);

	//give it 800 iterations, twice the default. 
	MSK_putintparam(task,MSK_IPAR_INTPNT_MAX_ITERATIONS,800);

	//----------------------------------

	//solve the thing
	r = MSK_optimize(task);
	
	//write problem to file
	/*
	static int fileNum = 0;
	if (r != MSK_RES_OK) {
		char filename[50];
		sprintf(filename,"mosek_error_%d_%d.opf",fileNum++, r);
		MSK_writedata(task, filename);
	}
	*/

	if (r != MSK_RES_OK) {
		DBGA("Mosek optimization call failed, error code " << r);
		MSK_deletetask(&task);
		return -1;
	}
	DBGP("Optimization complete");
	//debug code, find out number of iterations used
	//int iter;
	//MSK_getintinf(task, MSK_IINF_INTPNT_ITER, &iter); 
	//DBGA("Iterations used: " << iter);

	//find out what kind of solution we have
	MSKprostae pst;
	MSKsolstae sst;
	MSK_getsolutionstatus(task, MSK_SOL_ITR, &pst, &sst);
	int result;
	if (sst == MSK_SOL_STA_OPTIMAL || sst == MSK_SOL_STA_NEAR_OPTIMAL) {
		//success, we have an optimal problem
		if (sst == MSK_SOL_STA_OPTIMAL) {DBGP("QP solution is optimal");}
		else {DBGA("QP solution is *nearly* optimal");}
		result = 0;
	} else if (sst == MSK_SOL_STA_PRIM_INFEAS_CER) {
		//unfeasible problem
		DBGP("QP unfeasible");
		result = 1;
	} else if (sst == MSK_SOL_STA_PRIM_AND_DUAL_FEAS) {
		//i think this means feasible problem, but unbounded solution
		//this shouldn't happen as our Q is positive semidefinite
		DBGA("QP solution is prim and dual feasible, but not optimal");
		DBGA("Is Q positive semidefinite?");
		result = -1;
	} else {
		//unknown return status
		DBGA("QP fails with solution status " << sst << " and problem status " << pst);
		result = -1;
	}

	//MSK_SOL_STA_DUAL_FEAS;

	//retrieve the solutions
	if (!result) {
		//get the value of the objective function
		MSKrealt obj, foo;
		MSK_getsolutioninf(task, MSK_SOL_ITR, &pst, &sst, &obj,
						   &foo, &foo, &foo, &foo, &foo, &foo, &foo, &foo);
		*objVal = obj;
		double* xx = new double[sol.rows()];
		MSK_getsolutionslice(task, MSK_SOL_ITR, MSK_SOL_ITEM_XX,
							 0, sol.rows(), xx);
		for (int i=0; i<sol.rows(); i++) {
			sol.elem(i,0) = xx[i];
			DBGP("x" << i << ": " << xx[i]);
		}
		delete [] xx;	
	}
	MSK_deletetask(&task);
	return result;
}

/*! This version uses the factorization F of the matrix Q, where Q = FT * F
	See Mosek documentation, Sec. 7.3.2 */
/*
int mosekNNFactorizedQPSolverWrapper(const Matrix &F, 
									 const Matrix &Eq, const Matrix &b,
									 const Matrix &InEq, const Matrix &ib,
									 Matrix &sol)
{
	DBGP("Mosek QP Wrapper");
	MSKrescodee  r;
	MSKtask_t task = NULL; 

	// Create the mosek environment.
	MSKenv_t     env  = NULL;
	r = MSK_makeenv(&env,NULL,NULL,NULL,NULL);
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to create Mosek environment");
		return -1;
	}
    MSK_linkfunctoenvstream(env,MSK_STREAM_LOG,NULL,printstr);
	// Initialize the environment.
	r = MSK_initenv(env);
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to initialize Mosek environment");
		return -1;
	}
    // Create the optimization task.
    r = MSK_maketask(env, 0, 0, &task);
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to create optimization task");
		return -1;
	}
    MSK_linkfunctotaskstream(task,MSK_STREAM_LOG,NULL,printstr);

	//---------------------------------------
	//start inputing the problem
	//prespecify number of variables to make inputting faster

	//we have additional variables: y = F*x
	r = MSK_putmaxnumvar(task, sol.rows() + F.rows());

	//number of constraints (both equality and inequality)
	if (r == MSK_RES_OK) {
		//constraints on x
		int numCon = Eq.rows() + InEq.rows();
		//additional constraints on F*x - y = 0:
		numCon += F.rows();
		r = MSK_putmaxnumcon(task, numCon);
	}
	//number of non-zero entries in A
	//for now we assume both Eq and InEq are dense, although they might not be
	if (r == MSK_RES_OK) {
		//original constraints
		int numAnz = (Eq.rows() + InEq.rows())*Eq.cols();
		//additional constraints
		numAnz += F.rows() * ( F.cols() + F.rows() );
		r = MSK_putmaxnumanz(task,  );
	}
	if ( r!=MSK_RES_OK ) {
		DBGA("Failed to input variables");
		return -1;
	}

}*/
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
// $Id: mosek_qp.h,v 1.4.2.2 2009/04/27 14:33:16 cmatei Exp $
//
//######################################################################

/*! \file
	A wrapper for using the Mosek commercial QP solver from within GraspIt!
*/
class Matrix;

int mosekNNQPSolverWrapper(const Matrix &Q, 
						   const Matrix &Eq, const Matrix &b,
						   const Matrix &InEq, const Matrix &ib,
						   Matrix &sol, double *objVal, int numNN = -1);
/*
int mosekNNFactorizedQPSolverWrapper(const Matrix &F, 
									 const Matrix &Eq, const Matrix &b,
									 const Matrix &InEq, const Matrix &ib,
									 Matrix &sol);
 */
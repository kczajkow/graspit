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
// $Id: matrix.h,v 1.10.2.2 2009/04/27 14:33:12 cmatei Exp $
//
//######################################################################

#ifndef _matrix_h_
#define _matrix_h_

#include <assert.h>
#include <iostream>
#include <vector>
#include <list>

class transf;
class mat3;

//! A class for storing and performing operations on general 2-dimensional matrices
/*! The Matrix is a general 2-dimensional matrix, along with a (growing)
	number of operations that can be performed on it. The data itself is stored as
	a column-major array of doubles. The important design decisions are:

	A Matrix only holds doubles for now. At some point I hope to template the class 
	to hold	anything you might want.

	Many of the computations are done using LAPACK. This should be really fast,
	but also has problems. One of them is that is prevent using const correctly
	for all the operations and arguments that should be const.

	A Matrix's size is determined when it is constructed, and can never be changed
	afterwards. This is somewhat inflexible, but allows us to catch size 
	mismatches at run time. In debug mode, all operations first assert(...) the 
	correct matrix sizes.

	All Matrix copies are deep copies. This avoids lots of problem with consistency,
	but can also eat up memory and slow down operations. Be particularly careful 
	when using the copy constructor, or any functions that return instances of
	the Matrix class: all are done using deep copies.

	No operators have been implemented. Most operations that involve two or more
	matrices are external to this class, and rather than returning the result, they
	take in the result as an argument. This makes it somewhat easier to catch errors,
	but also make chaining of operations annoying.
*/
class Matrix{
private:
	int mRows, mCols;
	//! The actual data, in column-major format
	double *mData;
	//! Keeps track of row permutations that have been done
	int *mRowPermutations;
	//! Keeps track of column permutations that have been done
	int *mColPermutations;

	//! All the works that is common between all constructors
	void initialize(int m, int n);
	//! Copies the actual data from a column-major array in memory
	void setFromColMajor(const double *M);
	//! Copies the actual data from a row-major array in memory
	void setFromRowMajor(const double *M);
public:
	//! A matrix of the given size with unspecified contents
	Matrix(int m, int n);
	//! Copy constructor
	Matrix(const Matrix &M);
	//! A matrix of the given size, with contents initialized from an array in memory
	Matrix(const double *M, int m, int n, bool colMajor);
	virtual ~Matrix();

	inline double& elem(int m, int n);
	inline const double& elem(int m, int n) const;
	double *getData(){return mData;}
	void getData(std::vector<double> *data) const;
	int rows() const {return mRows;}
	int cols() const {return mCols;}
	Matrix getColumn(int c) const;
	Matrix getRow(int r) const;
	Matrix getSubMatrix(int startRow, int startCol, int rows, int cols) const;

	void setAllElements(double val);
	void copySubBlock(int startRow, int startCol, int rows, int cols, 
				      const Matrix &m, int startMRow, int startMCol);
	void copySubMatrix(int startRow, int startCol, const Matrix &m){
		copySubBlock(startRow, startCol, m.rows(), m.cols(), m, 0, 0);}
	void copyMatrix(const Matrix &m){copySubMatrix(0, 0, m);}

	friend std::ostream& operator<<(std::ostream &os, const Matrix &m);
	void print(FILE *fp = stderr) const;

	//! Computes the rank of the matrix using SVD
	int rank() const;
	//! Computes the Frobenius norm of the matrix sqrt(sum_of_all_squared_elems)
	double fnorm() const;
	//! The largest absolute value of any element in this matrix
	double absMax() const;
	void swapRows(int r1, int r2);
	void swapCols(int c1, int c2);
	//! Transposes this matrix in place
	void transpose();
	//! Sets this matrix to identity. Not really use, use the static EYE instead
	void eye();
	//! Multiples the matrix by the scalar s
	void multiply(double s);

	//! Returns the transpose of this matrix; the original is unaffected
	Matrix transposed() const;

	//! An identity matrix of the given size
	static Matrix EYE(int m, int n);
	//! A negated identity matrix of the given size
	static Matrix NEGEYE(int m, int n);
	//! A mtrix of the given size filled with zeroes
	static Matrix ZEROES(int m, int n);
	//! A permutation matrix initialized from a permutation vector in memory
	static Matrix PERMUTATION(int n, int *jpvt);
	//! A 4x4 transform matrix that can be used to left-multiply a 4x1 homogeneous vector
	static Matrix TRANSFORM(const transf &t);
	//! A 3x3 rotation matrix that can be used to left-multiply a 3x1 vector
	static Matrix ROTATION(const mat3 &rot);
	//! Builds a block diagonal matrix from the matrices in the list
	static Matrix BLOCKDIAG(std::list<Matrix*> *blocks);
	//! Builds a block column matrix from the column matrices in the list
	static Matrix BLOCKCOLUMN(std::list<Matrix*> *blocks);

	//! Used for zero comparisons in all computations
	static const double EPS;
};

//! Performs M = L * R
void matrixMultiply(const Matrix &L, const Matrix &R, Matrix &M);
//! Performs M = L + R
void matrixAdd(const Matrix &L, const Matrix &R, Matrix &M);
//! Checks if two matrices are identical
bool matrixEqual(const Matrix &R, const Matrix &L);
//! Solves the system A*X=B with square A. X is overwritten on B. 
int triangularSolve(Matrix &A, Matrix &B);
//! Computes minimum norm solution of underdetermined system A*X=B with full-rank A.
int underDeterminedSolveSVD(Matrix &A, Matrix &B, Matrix &X);
//! Computes minimum norm solution of underdetermined system A*X=B even for rank-deficient A
int underDeterminedSolveQR(Matrix &A, Matrix &B, Matrix &X);
//! Computes a solution of an underdetermined system A*X = B using Moore-Penrose pseudo-inverse
int underDeterminedSolveMPInv(Matrix &A, Matrix &B, Matrix &X);
//! Computes the inverse of a square matrix
int matrixInverse(const Matrix &A, Matrix &AInv);
//! Solves a Quadratic Program
int nonnegativeQPSolver(const Matrix &Q, 
						const Matrix &Eq, const Matrix &b,
						const Matrix &InEq, const Matrix &ib,
						Matrix &sol, bool doScaling, double *objVal, int numNN = -1);
//! Solves a factorized Quadratic Program
int nonnegativeFactorizedQPSolver(const Matrix &Qf, 
								  const Matrix &Eq, const Matrix &b,
								  const Matrix &InEq, const Matrix &ib,			
								  Matrix &sol, bool doScaling, double *objVal);
//! A simple tests to check that the QP solver works
void testQP();

double&
Matrix::elem(int m, int n) 
{
	assert(m<mRows);
	assert(n<mCols);
	return mData[n*mRows + m];
}

const double&
Matrix::elem(int m, int n) const
{
	assert(m<mRows);
	assert(n<mCols);
	return mData[n*mRows + m];
}

#endif

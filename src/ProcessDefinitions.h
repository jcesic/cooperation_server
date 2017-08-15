/*
 * ProcessDefinitions.h
 *
 *  Created on: Jan 24, 2014
 *      Author: andrej
 */

#ifndef PROCESSDEFINITIONS_H_
#define PROCESSDEFINITIONS_H_

#define SD 4	// state dimension
#define MD 7	// measurement dimension
#define DEG2RAD		(PI / 180.0)
#define RAD2DEG		(180.0 / PI)
#define M_EPS 1e-10

//#define CD 3;	// control inputs dimension

// fixed size matrices in process
typedef Eigen::Matrix<double,6,1> StateVector;
typedef Eigen::Matrix<double, 3, 1> TranVector;
typedef Eigen::Matrix<double, 3, 3> RotMat;
typedef Eigen::Matrix<double, 6, 6> StateMat;
typedef Eigen::Matrix<double, 4, 4> SE3Mat;

typedef Eigen::SparseMatrix<double> SparseMat;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatR;
typedef Eigen::Triplet<double> SparseTriplet;

#endif /* PROCESSDEFINITIONS_H_ */

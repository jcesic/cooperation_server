/*
 * UnscentedTransform.h
 *
 *  Created on: Nov 17, 2009
 *      Author: andreja
 */

#ifndef UNSCENTEDTRANSFORM_H_
#define UNSCENTEDTRANSFORM_H_

#include <Eigen/Eigen>
#include <Eigen/Cholesky>
#include <ProcessDefinitions.h>
using namespace Eigen;
class UT {
public:
	SE3Mappings SEMap;
	int UTCovTranAnglesToAxis(StateVector& x, StateMat& Px, StateVector& y, StateMat& Py) {

		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			StateVector tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTCovTranAnglesToAxis(tmp_block);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTCovTranAnglesToAxis(StateVector &x) {
		StateVector result;
		TranVector angles = x.block(3,0,3,1);
		TranVector t = x.block(0,0,3,1);
		TranVector axis = SEMap.angle2axis(angles);

		result.block(0,0,3,1) = t;
		result.block(3,0,3,1) = axis;
		return result;
	}

	int UTAnglesToAxis(TranVector& x, RotMat& Px, TranVector& y, RotMat& Py) {

		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			TranVector tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = SEMap.angle2axis(tmp_block);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	int UTGroupToAlgebra(StateVector& x, StateMat& Px, StateVector& y, StateMat& Py) {

		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			StateVector tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTGroupToAlgebra(tmp_block);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTGroupToAlgebra(StateVector &x) {
		StateVector result;
		TranVector angles = x.block(3,0,3,1);
		TranVector t = x.block(0,0,3,1);
		RotMat m_Rot = SEMap.angle2rot(angles);
		SE3Mat x_G;
		x_G.setIdentity();
		x_G.block(0,0,3,3) = m_Rot;
		x_G.block(0,3,3,1) = t;
		result = SEMap.log(x_G);
		return result;
	}

/*	int UTAlgebraToGroup(StateVector& x, StateMat& Px, StateVector& y, StateMat& Py) {

		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			StateVector tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTAlgebraToGroup(tmp_block);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTAlgebraToGroup(StateVector &x) {
		StateVector result;
		double fi = x(2,0);
		result(0) =(x(0,0)*sin(fi)+x(1,0)*(-1+cos(fi)))/fi;
		result(1) =(x(0,0)*(1-cos(fi))+x(1,0)*sin(fi))/fi;
		result(2) = fi;
		return result;
	}*/

	int UTPoseBetweenStates(VectorXd& x, MatrixXd& Px, StateVector& y, StateMat& Py, int dimPose) {
		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			VectorXd tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTPoseBetweenStates(tmp_block,dimPose);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTPoseBetweenStates(VectorXd &x,int dimPose) {
		StateVector x_11,x_22, result;
		SE3Mat x_11G, x_22G,x_12;
		x_11 = x.block(0,0,dimPose,1);
		x_22 = x.block(dimPose,0,dimPose,1);
		x_11G = SEMap.exp(x_11);
		x_22G = SEMap.exp(x_22);
		x_12 = x_11G.inverse()*x_22G;
		result = SEMap.logV(x_12);
		return result;
	}

	int UTPoseBetweenCoop(VectorXd& x, MatrixXd& Px, StateVector& y, StateMat& Py, int dimPose) {
		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			VectorXd tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTPoseBetweenStates(tmp_block,dimPose);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTPoseBetweenCoop(VectorXd &x,int dimPose) {
		StateVector x_11,x_22, result;
		SE3Mat x_11G, x_22G,x_12;
		x_11 = x.block(0,0,dimPose,1);
		x_22 = x.block(dimPose,0,dimPose,1);
		x_11G = SEMap.exp(x_11);
		x_22G = SEMap.exp(x_22);
		x_12 = x_11G.inverse()*x_22G;
		result = SEMap.logV(x_12);
		return result;
	}

/*	int UTPoseBetweenGroupStates(VectorXd& x, MatrixXd& Px, StateVector& y, StateMat& Py, int dimPose) {
		int L = x.rows();
		int nY = y.rows();

		// tunable parameters
		double alpha = 1e-3;
		double ki = 0;
		double beta = 2;

		double lambda = pow(alpha,2)*(L+ki)-L; // scaling factor
		double c = L+lambda; // scaling factor

		VectorXd wm; wm.resize(2*L+1);
		MatrixXd Wc; Wc.resize(2*L+1,2*L+1);
		wm(0) = lambda/c;
		int i;
		for (i = 1; i < 2*L+1; i++)
			wm(i) = 0.5/c;
		Wc.setIdentity();
		for (i = 0; i < 2*L+1; i++)
			Wc(i,i) = wm(i);
		Wc(0,0) += 1-pow(alpha,2)+beta;
		c = sqrt(c);

		MatrixXd Ls = Px.llt().matrixL();
		MatrixXd Chol = c * Ls;
		MatrixXd sigmaPoints(x.rows(),2*L+1);
		sigmaPoints.block(0,0,x.rows(),x.cols()) = x;
		for (i = 1; i <= L; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x + Chol.block(0, i-1, L, 1);
		for (i = L+1; i < 2*L+1; i++)
			sigmaPoints.block(0,i,x.rows(),x.cols()) = x - Chol.block(0, i-L-1, L, 1);
		y.setZero();
		MatrixXd tmpY(nY, 2*L+1);
		for (i = 0; i < 2*L+1; i++) {
			VectorXd tmp_block = sigmaPoints.block(0, i, L, 1);
			tmpY.block(0, i, nY, 1) = fUTPoseBetweenGroupStates(tmp_block,dimPose);
			y += wm(i)*tmpY.block(0, i, nY, 1);
		}
		for (i = 0; i < 2*L+1; i++)
			tmpY.block(0, i, nY, 1) = tmpY.block(0, i, nY, 1) - y;
		Py = tmpY * Wc* tmpY.transpose();
		return 0;
	}

	StateVector fUTPoseBetweenGroupStates(VectorXd &x,int dimPose) {
		StateVector x_11,x_22, result;
		StateMat x_11G, x_22G,x_12;
		x_11 = x.block(0,0,dimPose,1);
		x_22 = x.block(dimPose,0,dimPose,1);
		x_11G = SEMap.expV(x_11);
		x_22G = SEMap.expV(x_22);
		x_12 = x_11G.inverse()*x_22G;
		result = SEMap.logV(x_12);
		return result;
	}*/
};
#endif /* UNSCENTEDTRANSFORM_H_ */

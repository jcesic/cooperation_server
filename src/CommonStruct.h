#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)
#include <ProcessDefinitions.h>

using namespace Eigen;

#ifndef COMMON_STRUCT
#define COMMON_STRUCT
#define PI 3.14159265358979323846
#define DEG2RAD (PI / 180.0)

struct mat_params{
	int rows, cols, numbers;
};

struct LOOP_MATCHES{
	int index1, index2;
};

struct M_POSE_PLANE{
	Matrix3d m_Rot;
	Vector3d m_X, m_N;
	Vector4d m_q;
	double m_sa,m_ca,m_Alpha,m_Beta,m_Theta;
	double m_C[27];
	Matrix3d translation_C,angles_C,t_a_C;

	void UpdateRotFromQuat(){
		m_Rot(0,0) = m_q(0)*m_q(0)+m_q(1)*m_q(1)-m_q(2)*m_q(2)-m_q(3)*m_q(3);
		m_Rot(0,1) = 2.0*m_q(1)*m_q(2)-2.0*m_q(0)*m_q(3);
		m_Rot(0,2) = 2.0*m_q(1)*m_q(3)+2.0*m_q(0)*m_q(2);
		m_Rot(1,0) = 2.0*m_q(1)*m_q(2)+2.0*m_q(0)*m_q(3);
		m_Rot(1,1) = m_q(0)*m_q(0)-m_q(1)*m_q(1)+m_q(2)*m_q(2)-m_q(3)*m_q(3);
		m_Rot(1,2) = 2.0*m_q(2)*m_q(3)-2.0*m_q(0)*m_q(1);
		m_Rot(2,0) = 2.0*m_q(1)*m_q(3)-2.0*m_q(0)*m_q(2);
		m_Rot(2,1) = 2.0*m_q(2)*m_q(3)+2.0*m_q(0)*m_q(1);
		m_Rot(2,2) = m_q(0)*m_q(0)-m_q(1)*m_q(1)-m_q(2)*m_q(2)+m_q(3)*m_q(3);
	}

	void UpdatePTRLL(){
		m_Alpha = atan2(m_Rot(0,2), m_Rot(2,2));
		m_Beta = asin(-m_Rot(1,2));
		m_Theta = atan2(m_Rot(1,0), m_Rot(1,1));
	}

	void UpdateRotLL(){
		double ca = cos(m_Alpha);
		double sa = sin(m_Alpha);
		double cb = cos(m_Beta);
		double sb = sin(m_Beta);
		double cq = cos(m_Theta);
		double sq = sin(m_Theta);

		m_Rot(0,0) = ca*cq+sa*sb*sq;
		m_Rot(0,1) = -ca*sq+sa*sb*cq;
		m_Rot(0,2) = sa*cb;

		m_Rot(1,0) = cb*sq;
		m_Rot(1,1) = cb*cq;
		m_Rot(1,2) = -sb;

		m_Rot(2,0) = -sa*cq+ca*sb*sq;
		m_Rot(2,1) = sa*sq+ca*sb*cq;
		m_Rot(2,2) = ca*cb;
	}

	/*void setRVLPose(){
		m_Alpha = -m_Alpha;
		Vector3d tmpm_X = m_X;
		m_X(0)=-tmpm_X(1)*1000;
		m_X(2)=tmpm_X(0)*1000;
		UpdateRotLL();
		m_sa = sin(m_Alpha);
		m_ca = cos(m_Alpha);
	}

	void PoseSLAM2RVL(StateVector& VPose_1_2){
		m_Alpha = -VPose_1_2(2,0);
		m_X(0)=-VPose_1_2(1,0)*1000;
		m_X(2)=VPose_1_2(0,0)*1000;
		UpdateRotLL();
		m_sa = sin(m_Alpha);
		m_ca = cos(m_Alpha);
	}*/

	void PoseFromSE3Mat(SE3Mat &SE3Pose){
		m_Rot = SE3Pose.block(0,0,3,3);
		m_X = SE3Pose.block(0,3,3,1)*1000.0;
		UpdatePTRLL();
		m_sa = sin(m_Alpha);
		m_ca = cos(m_Alpha);
	}

	void CovSLAM2RVL(StateMat& P_12G){
		angles_C = P_12G.block(3,3,3,3);
		translation_C = P_12G.block(0,0,3,3)*1000.0*1000.0;
		t_a_C=P_12G.block(0,3,3,3)*1000.0;
	}

	/*void PoseRVL2SLAM(StateVector& zg){
		zg(0,0) = m_X(2)/1000.0;
		zg(1,0)= -m_X(0)/1000.0;
		zg(2,0) = -m_Alpha;
	}

	void CovRVL2SLAM(StateMat& z_C){
		z_C.setZero();
		z_C(0,0)=translation_C(2,2)/1000.0/1000.0;
		z_C(1,1)=translation_C(0,0)/1000.0/1000.0;
		z_C(2,2)=angles_C(0,0);
		z_C(0,1)=z_C(1,0)=translation_C(2,0)/1000.0/1000.0;
		z_C(0,2)=z_C(2,0)=t_a_C(0,2)/1000.0;
		z_C(1,2)=z_C(2,1)=t_a_C(0,0)/1000.0;
	}*/

	void Reset(){
		m_Rot.setIdentity();
		m_Alpha = m_Beta = m_Theta = 0.0;
		m_X.setZero();
		translation_C.setIdentity();
		angles_C.setIdentity();
		t_a_C.setZero();
	}
};
#endif

#ifndef SE3MAPPINGS_H
#define SE3MAPPINGS_H

#include <Eigen/Eigen>
using namespace Eigen;
#include "Common.h"
#include "ProcessDefinitions.h"
#include <iostream>
#include <stdio.h>

using namespace std;

class SE3Mappings{

public:
	StateMat adj(StateVector x){
		StateMat ret_value;
		TranVector t,w;
		RotMat tx,wx;
		t = x.block(0,0,3,1);
		w = x.block(3,0,3,1);
		tx = SE3x(t);
		wx = SE3x(w);
		ret_value.setZero();
		ret_value.block(0,0,3,3) = wx;
		ret_value.block(3,3,3,3) = wx;
		ret_value.block(0,3,3,3) = tx;
		return ret_value;
	}

	RotMat angle2rot (TranVector angles){
		RotMat m_Rot;
		double m_Alpha = angles(1,0);
		double m_Beta = angles(0,0);
		double m_Theta = angles(2,0);

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
		return m_Rot;
	}

	TranVector rot2angle(RotMat m_Rot){
		TranVector ret_value;
		ret_value(0,0) = asin(-m_Rot(1,2));
		ret_value(1,0) = atan2(m_Rot(0,2), m_Rot(2,2));
		ret_value(2,0) = atan2(m_Rot(1,0), m_Rot(1,1));
		return ret_value;
	}

	TranVector angle2axis(TranVector angles){
		double c1 = cos(angles(2,0)/2);
		double c2 = cos(angles(1,0)/2);
		double c3 = cos(angles(0,0)/2);
		double s1 = sin(angles(2,0)/2);
		double s2 = sin(angles(1,0)/2);
		double s3 = sin(angles(0,0)/2);

		double angle = 2 * acos(c1*c2*c3 - s1*s2*s3);

		double x = s1*s2*c3 + c1*c2*s3;
		double y = c1*s2*c3 - s1*c2*s3;
		double z = s1*c2*c3 +c1*s2*s3;

		double d = sqrt(x*x+y*y+z*z);
		if(d==0){
		    x = 0;
		    y = 0;
		    z = 0;
		}else{
		    x = x/d;
		    y = y/d;
		    z = z/d;
		}
		TranVector ret_value;
		ret_value(0,0)=x*angle;
		ret_value(1,0)=y*angle;
		ret_value(2,0)=z*angle;
		return ret_value;
	}

	StateMat Ad(SE3Mat x_G){
		RotMat R,tx;
		TranVector t;
		R = x_G.block(0,0,3,3);
		t = x_G.block(0,3,3,1);
		tx = SE3x(t);
		StateMat ret_value;
		ret_value.setZero();
		ret_value.block(0,0,3,3)=R;
		ret_value.block(3,3,3,3)=R;
		ret_value.block(0,3,3,3)=tx*R;
		return ret_value;
	}

	SE3Mat exp(StateVector x_g){
		TranVector u,w;
		RotMat wx,R,V,I;
		I.setIdentity();
		double A,B,C;
		u = x_g.block(0,0,3,1);
		w = x_g.block(3,0,3,1);
		wx = SE3x(w);
		double fi = sqrt(w.transpose()*w);
		if(fi==0){
			fi = pow(10,-10);
		}

		A = sin(fi)/fi;
		B = (1-cos(fi))/pow(fi,2);
		C= (1-A)/pow(fi,2);

		R = I + A*wx + B*(wx*wx);
		V = I + B*wx + C*(wx*wx);

		SE3Mat ret_value;
		ret_value.setIdentity();
		ret_value.block(0,0,3,3)=R;
		ret_value.block(0,3,3,1)=V*u;
		return ret_value;
	}

	RotMat SE3x(TranVector x){
		RotMat ret_value;
		ret_value.setZero();
		ret_value(0,1)=-x(2,0);
		ret_value(1,0)=x(2,0);
		ret_value(0,2)=x(1,0);
		ret_value(2,0)=-x(1,0);
		ret_value(1,2)=-x(0,0);
		ret_value(2,1)=x(0,0);
		return ret_value;
	}

	TranVector invHat(RotMat x){
		TranVector ret_value;
		ret_value.setZero();
		ret_value(0,0)=x(2,1);
		ret_value(1,0)=x(0,2);
		ret_value(2,0)=x(1,0);
		return ret_value;
	}

	StateVector log(SE3Mat x_G){
		RotMat R,I,lnR,wx,invV;
		TranVector t,u,w;
		double fi,A,B;
		I.setIdentity();
		R = x_G.block(0,0,3,3);
		t = x_G.block(0,3,3,1);
		if(R==I){
			u = t;
			w.setZero();
		}else{
			fi = acos((R.trace()-1)/2);

			if(fi==0){
				fi = pow(10,-5);
			}

			lnR = fi/(2*sin(fi))*(R-R.transpose());
			w = invHat(lnR);
			wx = SE3x(w);
			A = sin(fi)/fi;
			B = (1-cos(fi))/pow(fi,2);
			invV = I - 0.5*wx + 1/pow(fi,2)*(1-A/(2*B))*wx*wx;
			u = invV*t;
		}
		StateVector ret_value;
		ret_value.setZero();
		ret_value.block(0,0,3,1)=u;
		ret_value.block(3,0,3,1)=w;
		return ret_value;
	}

	StateVector logV(SE3Mat x_G){
		TranVector t;
		RotMat R;
		t = x_G.block(0,3,3,1);
		R = x_G.block(0,0,3,3);
		StateVector ret_value;
		ret_value.setZero();
		ret_value.block(0,0,3,1)=t;
		ret_value.block(3,0,3,1)=rot2angle(R);
		return ret_value;
	}

	StateMat PhiG(StateMat adG){
		int k = 10;
		StateMat ret_value;
		ret_value.setZero();
		int fac_i=1;
		StateMat pow_ad;
		pow_ad.setIdentity();
		for(int i = 0; i < k; i++){
			fac_i = fac_i*(i+1);
			ret_value=ret_value+((pow(-1,i)/1.0/(double)fac_i)*pow_ad);
			pow_ad = pow_ad*adG;
		}
		return ret_value;
	}
};
#endif

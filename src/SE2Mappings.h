#include <Eigen/Eigen>
using namespace Eigen;
#include "Common.h"
#include "ProcessDefinitions.h"

class SE2Mappings{

public:
	StateMat adj(StateVector x){
		StateMat ret_value;
		ret_value.setZero();
		ret_value(0,1) = -x(2,0);
		ret_value(1,0) = x(2,0);
		ret_value(0,2) = x(1,0);
		ret_value(1,2) = -x(0,0);
		return ret_value;
	}

	StateMat Ad(StateMat x_G){
		StateMat ret_value;
		ret_value.setZero();
		ret_value(0,0) = x_G(0,0);
		ret_value(0,1) = x_G(0,1);
		ret_value(1,0) = x_G(1,0);
		ret_value(1,1) = x_G(1,1);
		ret_value(0,2) = x_G(1,2);
		ret_value(1,2) = -x_G(0,2);
		ret_value(2,2) = 1;
		return ret_value;
	}

	StateMat exp(StateMat x_g){
		StateMat ret_value;
		ret_value.setZero();
		double fi = x_g(1,0);
		if (fi==0){
			fi = pow(10,-10);
		}
		double tGx =(x_g(0,2)*sin(fi)+x_g(1,2)*(-1+cos(fi)))/fi;
		double tGy =(x_g(0,2)*(1-cos(fi))+x_g(1,2)*sin(fi))/fi;
		ret_value(0,0) = cos(fi);
		ret_value(0,1) = -sin(fi);
		ret_value(1,0) = sin(fi);
		ret_value(1,1) = cos(fi);
		ret_value(0,2) = tGx;
		ret_value(1,2) = tGy;
		ret_value(2,2) = 1;
		return ret_value;
	}

	StateVector hatInv(StateMat x_g){
		StateVector ret_value;
		ret_value(0,0) = x_g(0,2);
		ret_value(1,0) = x_g(1,2);
		ret_value(2,0) = x_g(1,0);
		return ret_value;
	}

	StateMat hat(StateVector x){
		StateMat ret_value;
		ret_value.setZero();
		ret_value(0,1) = -x(2,0);
		ret_value(0,2) = x(0,0);
		ret_value(1,0) = x(2,0);
		ret_value(1,2) = x(1,0);
		return ret_value;
	}

	StateMat log(StateMat x_G){
		StateMat ret_value;
		ret_value.setZero();
		double fi = atan2(x_G(1,0),x_G(0,0));
		if (fi==0)
			fi = pow(10,-10);
		double A = sin(fi)/fi;
		double B = (1-cos(fi))/fi;
		MatrixXd tmp(2,2);
		tmp << A, B, -B, A;
		MatrixXd invV = 1/(A*A+B*B)*tmp;
		tmp << 0, -1, 1, 0;
		ret_value.block(0,0,2,2)=fi*tmp;
		VectorXd tmpv(2);
		tmpv << x_G(0,2), x_G(1,2);
		ret_value.block(0,2,2,1) = invV*tmpv;
		return ret_value;
	}

	StateVector logV(StateMat x_G){
		StateVector ret_value;
		ret_value(0,0) =x_G(0,2);
		ret_value(1,0)=x_G(1,2);
		ret_value(2,0)=atan2(x_G(1,0),x_G(0,0));
		return ret_value;
	}

	StateMat expV(StateVector x_g){
		StateMat ret_value;
		ret_value.setIdentity();
		ret_value(0,0)=ret_value(1,1)=cos(x_g(2));
		ret_value(0,1)=-sin(x_g(2));
		ret_value(1,0)=sin(x_g(2));
		ret_value(0,2)=x_g(0);
		ret_value(1,2)=x_g(1);
		return ret_value;
	}

	StateMat PhiG(StateMat adG){
		int k = 10;
		StateMat ret_value;
		ret_value.setZero();
		int fac_i=1;
		MatrixXd pow_ad(3,3);
		pow_ad.setIdentity();
		for(int i = 0; i < k; i++){
			fac_i = fac_i*(i+1);
			ret_value=ret_value+((pow(-1,i)/1.0/(double)fac_i)*pow_ad);
			pow_ad = pow_ad*adG;
		}
		return ret_value;
	}
};

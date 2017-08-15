#include "SLAM.h"
#include "ESDSF.h"

bool SLAM::add_ex_state(rvl_psulmdemo::predict_trace::Request &req, rvl_psulmdemo::predict_trace::Response &res){
	int state_id = req.stateID;
	printf("	REQUEST INFORMATION GAIN: %d\n",state_id);
	globalModel.esdsf.recoverState();
	VectorXd xt_1 = globalModel.esdsf.getRecoveredX().block(state_id*globalModel.esdsf.getDim(), 0, globalModel.esdsf.getDim(), 1);
	nav_msgs::Odometry odom_state;
	odom_state.pose.pose.position.x = xt_1(0);
	odom_state.pose.pose.position.y=xt_1(1);
	odom_state.pose.pose.orientation.w = xt_1(2);
	odom_state.pose.pose.orientation.z = xt_1(3);
	odom_state.pose.pose.orientation.x = 0.0;
	odom_state.pose.pose.orientation.y = 0.0;

	// pomaci u mm
	u(0) = odom_state.pose.pose.position.x - lastOdometryPose.position.x*1000;
	u(1) = odom_state.pose.pose.position.y - lastOdometryPose.position.y*1000;
	//			printf("%f %f %f %f %d\n",xt_1(0),xt_1(1),xt_1(2),xt_1(3),globalModel.esdsf.getDim());
	// logika da se izbjegne prekid Eulerovog kuta u PI
	double kut1 = tf::getYaw(odom_state.pose.pose.orientation);
	double kut2 = tf::getYaw(lastOdometryPose.orientation);
	double last_odometry_x = lastOdometryPose.position.x;
	double last_odometry_y = lastOdometryPose.position.y;
	ros::Time last_odometry_time = last_time;

	if (kut1-kut2 > PI) {
		u(2) = kut1 - _2PI -kut2;
	}
	else if(kut1-kut2 < -PI) {
		u(2) = kut1 - kut2 + _2PI;
	}
	else {
		u(2) = kut1-kut2;
	}

	//printf("u: %f %f %f\n",u(0),u(1),u(2));
	current_time = ros::Time::now();
	ros::Duration d = current_time-last_time;
	T = d.sec+d.nsec/1e9;

	//			void PathAndMap::discreteLinearMotionModel(StateVector& x, StateMat& P, ControlVector& u, double T, StateMat& F, StateMat& Q) {
	StateVector xt_tmp = xt;
	StateMat Pxx_tmp = Pxx;
	StateMat F_tmp = F;
	StateMat Q_tmp = Q;

	//			xt.print("xt_prije:\n");
	//			xt_tmp.print("xt_tmp prije: \n");
	globalModel.discreteLinearMotionModel(xt_tmp, Pxx_tmp, u, T, F_tmp, Q_tmp);
	//			xt.print("xt pos:\n");
	//			xt_tmp.print("xt_tmp pos:\n");

	if (Q_tmp.determinant() < 1e-12)
		globalModel.addProcessNoise(Pxx_tmp, Q_tmp);

	globalModel.esdsf.augmentState_tmp(xt_tmp, F_tmp, Q_tmp);
	globalModel.esdsf.recoverState_tmp();

	globalModel.esdsf.getRecoveredX_tmp().print("X after a: \n");
	MatrixXd Linv;
	globalModel.esdsf.inverseL(Linv);
	double trace_P=0;
	for(int i = 0; i < Linv.cols(); i++){
		trace_P=trace_P+sqrt(Linv(i,i));
	}
	res.trace_before = trace_P;
	printf("		P TRACE before: %d %d %f\n",Linv.cols(),Linv.rows(),trace_P);

	printf("		VISUAL UPDATE between indexes %d %d started\n",state_id,globalModel.esdsf.getN().rows()/4-1);

	int dimPose = globalModel.esdsf.getDim();
	VectorXd xi = globalModel.esdsf.getRecoveredX_tmp().block(state_id * dimPose, 0, dimPose, 1);
	VectorXd xj = globalModel.esdsf.getRecoveredX_tmp().block((globalModel.esdsf.getN_tmp().rows()/4-1) * dimPose, 0, dimPose, 1);

	double hm[7] = {xi(2)*xi(3)*(-2*xi(0) + 2*xj(0)) + (-1 + 2*pow(xi(2),2))*(xi(1) - xj(1)),
			0,
			xi(0)*(1 - 2*pow(xi(2),2)) + (-1 + 2*pow(xi(2),2))*xj(0) + 2*xi(2)*xi(3)*(-xi(1) + xj(1)),xi(2)*xj(2) + xi(3)*xj(3),
			0,
			xi(3)*xj(2) - xi(2)*xj(3),
			0};

	double Hm[7][8] = {{-2*xi(2)*xi(3),-1 + 2*pow(xi(2),2),
			xi(3)*(-2*xi(0) + 2*xj(0)) + 4*xi(2)*(xi(1) - xj(1)),
			xi(2)*(-2*xi(0) + 2*xj(0)),2*xi(2)*xi(3),
			1 - 2*pow(xi(2),2),0,0},{0,0,0,0,0,0,0,0},
			{1 - 2*pow(xi(2),2),-2*xi(2)*xi(3),
					-4*xi(0)*xi(2) + 4*xi(2)*xj(0) +
					2*xi(3)*(-xi(1) + xj(1)),2*xi(2)*(-xi(1) + xj(1)),
					-1 + 2*pow(xi(2),2),2*xi(2)*xi(3),0,0},
					{0,0,xj(2),xj(3),0,0,xi(2),xi(3)},
					{0,0,0,0,0,0,0,0},
					{0,0,-xj(3),xj(2),0,0,xi(3),-xi(2)},
					{0,0,0,0,0,0,0,0}};

	z0.setFromArray(hm);
	Hij.setFromArray(Hm[0]);

	// quaternion -q equivalent q
	if ( z0(3) < 0 ) {
		z0.block(3,0,4,1) = -z0.block(3,0,4,1);
		Hij.block(3,0,4,8) = -Hij.block(3,0,4,8);
	}

	z(0) = 0.0; // xc
	z(1) = 0.0; // yc
	z(2) = 0.0; // zc
	z(3) = 1;
	z(4) = 0.0;
	z(5) = 0.0;
	z(6) = 0.0;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			if(i==j){
				R(i,j)=100;
			}else{
				R(i,j) = 9;
			}
		}
	}
	for (int i = 3; i < 7; i++){
		for (int j = 3; j < 7; j++){
			if(i==j){
				R(i,j)=1.0e-06;
			}else{
				R(i,j) = 1.0e-8;
			}
		}
	}
	//			printf("%f %f %f %f %f %f %f\n",xi(0),xi(1),xj(0),xj(1),z0(0),z0(1),z0(2));

	H.resize(7, globalModel.esdsf.getN_tmp().rows());
	H.setBlock(0, 4 * state_id, Hij.block(0, 0, 7, 4));
	H.setBlock(0, 4 * (globalModel.esdsf.getN_tmp().rows()/4-1), Hij.block(0, 4, 7, 4));
	globalModel.esdsf.setMeasurIndexes(state_id, globalModel.esdsf.getN_tmp().rows()/4-1);

	globalModel.esdsf.update_tmp(z, z0, H, R);
	globalModel.esdsf.recoverState_tmp();
	globalModel.esdsf.inverseL(Linv);

	trace_P=0;
	for(int i = 0; i < Linv.cols(); i++){
		trace_P=trace_P+sqrt(Linv(i,i));
	}
	printf("		P TRACE AFTER: %d %d %f\n",Linv.cols(),Linv.rows(),trace_P);
	res.trace_after = trace_P;
	/*			pFile2 = fopen("/home/kruno/traces_P.txt","a");
	fprintf(pFile2,"%d %d %f %f\n",req.stateID,globalModel.esdsf.getN().rows()/4-1,res.trace_before,res.trace_after);
	fclose(pFile2);
	 */
	//globalModel.esdsf.getRecoveredX_tmp().print("X after u: \n");
	printf("	REQUEST COMPLETE!\n");
	return true;
}

void ESDSF::augmentState_tmp(StateVector& xt, const StateMat& F, const StateMat& Q) {
	// augmentation in information form
	StateMat Qinv = Q.inverse();
	MatrixXd R(dimPose, dimPose);
	int N = n.rows();
	int Nx = x.rows();
	L_tmp.resize(L.rows(),L.cols());
	L_tmp = L;
	n_tmp.resize(N);
	n_tmp = n;

	L_tmp.conservativeResize(N+dimPose, N+dimPose);
	n_tmp.conservativeResize(N+dimPose, 1);

	VectorXd xt_1 = x.block(Nx-dimPose, 0, dimPose, 1);

	R = StateMat(L_tmp.block(L_tmp.rows()-2*dimPose, L_tmp.cols()-2*dimPose, dimPose, dimPose)) + F.transpose()*Qinv*F;

	L_tmp.setBlock(L_tmp.rows()-2*dimPose, L_tmp.cols()-2*dimPose, R);
	R = -Qinv*F;
	L_tmp.setBlock(L_tmp.rows()-dimPose, L_tmp.cols()-2*dimPose, R);
	L_tmp.setBlock(L_tmp.rows()-2*dimPose, L_tmp.cols()-dimPose, R.transpose());
	L_tmp.setBlock(L_tmp.rows()-dimPose, L_tmp.cols()-dimPose, Qinv);

	//R.resize(dimPose, 1);

	R = n_tmp.block(n_tmp.rows()-2*dimPose, 0, dimPose, 1) - F.transpose()*Qinv*(xt-F*xt_1);
	n_tmp.block(n_tmp.rows()-2*dimPose, 0, R.rows(), R.cols()) = R;
	R = Qinv*(xt-F*xt_1);
	n_tmp.block(n_tmp.rows()-dimPose, 0, R.rows(), R.cols()) = R;
}

void ESDSF::update_tmp(const MeasurementVector& z, const MeasurementVector& h_k, const SparseMat& H_k, const MeasurMat& R_k) {
	MatrixXd temp = H_k.transpose()*R_k.inverse();
	n_tmp += temp * (z - h_k + H_k * X_tmp); // aliasing not present for component-wise operations
	MatrixXd dL = temp * H_k;

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			L_tmp.setBlock(index(i)*dimPose, index(j)*dimPose,
					StateMat(L_tmp.block(index(i)*dimPose, index(j)*dimPose, dimPose, dimPose))+dL.block(index(i)*dimPose, index(j)*dimPose, dimPose, dimPose));
}

void ESDSF::normalizeQuaternionsDisplOffNoP_tmp() {
	double norm;
	int idxStart = (dimPose == 4? 2:3); // 2D or 3D case
	int idxEnd = (dimPose == 4? 2:4);

	for (int i = idxStart; i <= X_tmp.rows()-idxEnd; i += dimPose) {

		norm = sqrt(X_tmp(i)*X_tmp(i) + X_tmp(i+1)*X_tmp(i+1));

		// -q equivalent q
		if (X_tmp(i) >= 0) {

			X_tmp(i) *= 1/norm;
			X_tmp(i+1) *= 1/norm;
			if (dimPose == 7) {
				X_tmp(i+2) *= 1/norm;
				X_tmp(i+3) *= 1/norm;
			}

		} else {

			X_tmp(i) *= -1/norm;
			X_tmp(i+1) *= -1/norm;
			if (dimPose == 7) {
				X_tmp(i+2) *= -1/norm;
				X_tmp(i+3) *= -1/norm;
			}
		}

	}

	// sparse multiplication
	n_tmp = L_tmp * X_tmp;
}

void ESDSF::recoverState_tmp() {

	// solve sparse SPD linear system: L*x = n
	//printf("ZAPOCINJEM POVRAT:  \n");
	//L.print("L prije povrata: \n");

	solver_tmp.compute(L_tmp); // performs a Cholesky factorization of L, L must be SPD
	//L.print("L prije povrta: \n");

	if(solver_tmp.info()!= Eigen::Success) {
		// solving failed
		printf("Decomposition failed\n");
	} else {
		 //X = solver.solve(n); // use the factorization to solve for the given right hand side
		//printf("1 %f\n", n(n.rows()-1));
		//printf("2 %f\n",n(n.rows()-2));

		//n.print("n prije povrata: \n");

		X_tmp = solver_tmp.solve(n_tmp);
		 // Print the result:
		//std::cout << "x = \n" << x << std::endl;
		//x.printToFile("x.txt", NULL);
	}

	normalizeQuaternionsDisplOffNoP_tmp();
	//X.print("X nakon normalizacije \n");
	return;
}


void ESDSF::inverseL(MatrixXd& Linv) {
	Linv.resize(L_tmp.rows(),L_tmp.cols());
	Eigen::SimplicialCholesky<SparseMatrix<double>, Eigen::Upper> solver_inverse;
	solver_inverse.compute(L_tmp);
	MatrixXd I;
	I.resize(L_tmp.cols(),L_tmp.rows());
	I.setIdentity();
	Linv = solver_tmp.solve(I);
}

#include "LESDSF.h"
#include "UnscentedTransform.h"

void LESDSF_class::Init(){
	dimPose = 6;

	/*	Kssx =0.11/1000.0;
		Kssy = 100*0.11/1000.0;
		Kssz = 1.5*0.11/1000.0;
		Kalpha = 0.005*PI/180 * PI/180;
		Kbeta = 0.005*PI/180 * PI/180;
		Ktheta = 0.005*PI/180 * PI/180;*/

	Kssx =0.4/1000.0;
	Kssy = 0.4/1000.0;
	Kssz = 0.4/1000.0;
	Kalpha = 0.005*PI/180 * PI/180;
	Kbeta = 0.005*PI/180 * PI/180;
	Ktheta = 0.005*PI/180 * PI/180;


	/*Kssx =30/1000.0;
	Kssy = 30/1000.0;
	Kssz = 30/1000.0;
	Kalpha = 20*PI/180 * PI/180;
	Kbeta = 20*PI/180 * PI/180;
	Ktheta = 20*PI/180 * PI/180;*/


	marg_id = 0;

	/*	Kssx =0.08/1000.0;
	Kssy = 0.08/1000.0;
	Kssz = 0.08/1000.0;
	Kalpha = 5*PI/180 * PI/180;
	Kbeta = 5*PI/180 * PI/180;
	Ktheta = 5*PI/180 * PI/180;*/
	/*
	Kssx =4/1000.0;
	Kssy = 4/1000.0;
	Kssz = 4/1000.0;
	Kalpha = 0.1*PI/180 * PI/180;
	Kbeta = 0.1*PI/180 * PI/180;
	Ktheta = 0.1*PI/180 * PI/180;*/

	minQnoise = pow(10,-8);
	minQnoiseLoop = minQnoise;//pow(10,-5);
	xt.setZero();
	Pxx.setIdentity();
	Q.setIdentity();
	Pxx = Pxx*minQnoise;
	Q = Q*minQnoise;
	x.resize(dimPose);
	x.setZero();
	xtG = SEMap.exp(xt);
	L.resize(dimPose,dimPose);
	MatrixXd Pinv = Pxx.inverse();
	L.setBlock(0, 0, Pinv);
	n = L*x;

	total_updates = num_p = num_poses = num_angles=0;
	t_glob_solve = 0.0;
	states_loops.clear();
}

void LESDSF_class::RecoverState(){
	solver.compute(L);
	X.resize(L.rows(),1);
	if(solver.info()!= Eigen::Success) {
		printf("Decomposition failed\n");
	} else {
		X = solver.solve(n);
	}
}

void LESDSF_class::getRelPose(int id1, int id2, StateVector& vPose){
	StateVector x1g = X.block(id1*dimPose,0,dimPose,1);
	StateVector x2g = X.block(id2*dimPose,0,dimPose,1);
	SE3Mat x1G = SEMap.exp(x1g);
	SE3Mat x2G = SEMap.exp(x2g);
	SE3Mat Pose = x1G.inverse()*x2G;
	vPose = SEMap.logV(Pose);
}

void LESDSF_class::StateCov(int id, StateMat& P ){
	solver.compute(L);
	VectorXd e_12(n.rows()), p_12;
	MatrixXd _P(n.rows(), dimPose);
	for (int k = id; k < id+dimPose; k++) {
		e_12.setZero();
		e_12(k) = 1;
		p_12 = solver.solve(e_12);
		_P.col(k-id) = p_12;
	}
	P.block(0, 0, dimPose, dimPose) = _P.block(id, 0, dimPose, dimPose); // Pii
}

void LESDSF_class::RelativeCov(int id1, int id2, MatrixXd& P_12 ){
	/* solve sparse SPD system
	 * L * x = n
	 * L * _P = I
	 */

	VectorXd e_12(n.rows()), p_12;
	MatrixXd _P(n.rows(), dimPose);
	for (int k = id1; k < id1+dimPose; k++) {
		e_12.setZero();
		e_12(k) = 1;
		p_12 = solver.solve(e_12);
		_P.col(k-id1) = p_12;
	}
	P_12.block(0, 0, dimPose, dimPose) = _P.block(id1, 0, dimPose, dimPose); // Pii
	P_12.block(dimPose, 0, dimPose, dimPose) = _P.block(id2, 0, dimPose, dimPose); // Pji
	StateMat Pij = P_12.block(dimPose, 0, dimPose, dimPose).transpose(); // Pij
	P_12.block(0, dimPose, dimPose, dimPose) = Pij;
	for (int k = id2; k < id2+dimPose; k++) {
		e_12.setZero();
		e_12(k) = 1;
		p_12 = solver.solve(e_12);
		_P.col(k-id2) = p_12;
	}
	P_12.block(dimPose, dimPose, dimPose, dimPose) = _P.block(id2, 0, dimPose, dimPose); // Pjj
}

void LESDSF_class::MotionModel(StateVector& Om){
	StateMat Qu, PhiOm, ut_Qu;
	RotMat Qangles,ut_Qangles;
	SE3Mat exp_Om,expOm_;
	TranVector angles,axis, ut_axis;
	StateVector Om_a,ut_tranAxis;

	angles = Om.block(3,0,3,1);
	axis = SEMap.angle2axis(angles);
	Om_a.block(0,0,3,1)=Om.block(0,0,3,1);
	Om_a.block(3,0,3,1) = axis;

	Qu.setZero();
#ifndef USE_ODOM_COVARIANCE
	Qu(0,0)=Kssx*fabs(Om(0));
	Qu(1,1)=Kssy*fabs(Om(1));
	Qu(2,2)=Kssz*fabs(Om(2));
	Qu(3,3)=Kalpha*fabs(Om(3));
	Qu(4,4)=Kbeta*fabs(Om(4));
	Qu(5,5)=Ktheta*fabs(Om(5));
#else
	for (int i = 0; i < 6; i++){
		for(int j = 0; j < 6; j++){
			Qu(i,j)=Qu_test(i,j);
		}
	}
	//cout << "Qu" << endl << Qu << endl;
	//cout << "Qu_test" << endl << Qu_test << endl;
#endif

	for (int i = 0; i < 6; i++){
		if(Qu(i,i)<minQnoise){
			Qu(i,i)=minQnoise;
		}
	}

	//Qangles = Qu.block(3,3,3,3);
	UT ut;
	//ut.UTAnglesToAxis(angles,Qangles,ut_axis,ut_Qangles);
	ut.UTCovTranAnglesToAxis(Om,Qu,ut_tranAxis,ut_Qu);
	//Qu.block(3,3,3,3)=ut_Qangles;

	exp_Om = SEMap.exp(Om_a);
	expOm_ = SEMap.exp(-Om_a);

	F=SEMap.Ad(expOm_);
	PhiOm = SEMap.PhiG(SEMap.adj(Om_a));
	Q = PhiOm*ut_Qu*PhiOm.transpose();

	xtG = xtG*exp_Om;
	Pxx = (F*Pxx*F.transpose()+Q);
}

void LESDSF_class::write_to_shared(int agent_id){
	char buffer_L[100], buffer_P[100];
	snprintf(buffer_L, sizeof(buffer_L), "L_mat_%d", agent_id);
	snprintf(buffer_P, sizeof(buffer_P), "mat_params_%d", agent_id);

	boost::interprocess::shared_memory_object::remove(buffer_L);
	boost::interprocess::shared_memory_object::remove(buffer_P);

	boost::interprocess::shared_memory_object L_shm (boost::interprocess::create_only, buffer_L, boost::interprocess::read_write);
	boost::interprocess::shared_memory_object par_shm (boost::interprocess::create_only, buffer_P, boost::interprocess::read_write);

	int brojac=0;
	std::vector<EigenT> tripletList;
	for (int k=0; k<L.outerSize(); ++k){
	  for (SparseMatrix<double>::InnerIterator it(L,k); it; ++it){
		tripletList.push_back(EigenT(it.row(),it.col(),it.value()));
		brojac++;
	  }
	}
	mat_params L_P;
	L_P.numbers = brojac;
	L_P.rows = L.rows();
	L_P.cols = L.cols();

	L_shm.truncate(brojac*sizeof(EigenT));
	par_shm.truncate(sizeof(L_P));

	boost::interprocess::mapped_region L_region(L_shm, boost::interprocess::read_write);
	memcpy(L_region.get_address(),&(*tripletList.begin()),tripletList.size()*sizeof(EigenT));

	boost::interprocess::mapped_region par_region(par_shm, boost::interprocess::read_write);
	memcpy(par_region.get_address(),&L_P,sizeof(L_P));
}

void LESDSF_class::AugmentState() {
	xt = SEMap.log(xtG);
	// augmentation in information form
	StateMat Qinv = Q.inverse();
	MatrixXd R(dimPose, dimPose);
	int N = n.rows();
	int Nx = x.rows();
	L.conservativeResize(N+dimPose, N+dimPose);
	n.conservativeResize(N+dimPose, 1);

	VectorXd xt_1 = x.block(Nx-dimPose, 0, dimPose, 1);

	x.conservativeResize(Nx+dimPose, 1);

	R = StateMat(L.block(L.rows()-2*dimPose, L.cols()-2*dimPose, dimPose, dimPose)) + F.transpose()*Qinv*F;

	L.setBlock(L.rows()-2*dimPose, L.cols()-2*dimPose, R);
	R = -Qinv*F;
	L.setBlock(L.rows()-dimPose, L.cols()-2*dimPose, R);
	L.setBlock(L.rows()-2*dimPose, L.cols()-dimPose, R.transpose());
	L.setBlock(L.rows()-dimPose, L.cols()-dimPose, Qinv);

	R = n.block(n.rows()-2*dimPose, 0, dimPose, 1) - F.transpose()*Qinv*(xt-F*xt_1);
	n.block(n.rows()-2*dimPose, 0, R.rows(), R.cols()) = R;
	R = Qinv*(xt-F*xt_1);
	n.block(n.rows()-dimPose, 0, R.rows(), R.cols()) = R;

	x.block(x.rows()-dimPose, 0, xt.rows(), xt.cols()) = xt;
	if (Nx >= 2*dimPose) { // marginalizacija 0-tog stanja, tako da x sadrzi trenutni i prethodni polozaj
		x = x.block(dimPose, 0, 2*dimPose, 1).eval(); // anti-aliasing with eval for temp. storage
	}
}

gain_ret LESDSF_class::InfGainLxx(int src, int dest, SparseMat& LXS){
	gain_ret weight;
	StateMat Ljj = LXS.block(src*dimPose,src*dimPose,dimPose,dimPose);
	StateMat Lii = LXS.block(dest*dimPose,dest*dimPose,dimPose,dimPose);
	StateMat Lij = LXS.block(dest*dimPose,src*dimPose,dimPose,dimPose);
	StateMat Lji = Lij.transpose();
	StateMat ILjj = Ljj.inverse();
	StateMat R = Lii - Lij*ILjj*Lji;
	weight.Inf = 0.5*log2(Lii.determinant()/R.determinant());
	weight.DLii = Lii.determinant();
	weight.IDLjj = ILjj.determinant();
	return weight;
}

gain_ret LESDSF_class::InfGainRS(int src, int dest, SparseMat& LXS, SparseMat& RS, SparseMat &LZS){
	gain_ret weight;
	StateMat Ljj = LXS.block(src*dimPose,src*dimPose,dimPose,dimPose);
	StateMat Lii = LZS.block(dest*dimPose,dest*dimPose,dimPose,dimPose);
	StateMat Lji = RS.block(src*dimPose,dest*dimPose,dimPose,dimPose);
	StateMat Lij = Lji.transpose();

	StateMat ILjj = Ljj.inverse();
	StateMat R = Lii - Lij*ILjj*Lji;
	weight.Inf = 0.5*log2(Lii.determinant()/R.determinant());
	weight.DLii = Lii.determinant();
	weight.IDLjj = ILjj.determinant();
	return weight;
}

void LESDSF_class::MarginalizeS(bool recover_state, int state_IDm, int state_IDn, vector<StateVector> &slam_trajectory) {
	// augmentation with current state (xt, Pxx) and marginalization over the past state
	// in information form
	int N = n.rows();
	int Nx = x.rows();
	xt = SEMap.log(xtG);

	printf("Block marginalization: %d %d %ld\n",state_IDm,state_IDn,n.rows()/dimPose);
	if(N>2*dimPose){
		ros::Time t_ros_total = ros::Time::now();
		std::vector<SparseTriplet> tripletList,tripletListLyy;
		int state_diff = state_IDn - state_IDm + 1;
		int state_IDn1 = state_IDn+1;
		ros::Time t_ros_recover = ros::Time::now();
		if(recover_state){
			slam_trajectory.push_back(x.block(dimPose,0,dimPose,1));
			X.resize(slam_trajectory.size()*dimPose);
			for (int i = 0; i < slam_trajectory.size(); i++){
				X.block(i*dimPose,0,dimPose,1) = slam_trajectory[i];
			}
		}
		ros::Duration d_recover = ros::Time::now() - t_ros_recover;

		ros::Time t_ros_init = ros::Time::now();
		printf("Setting Lyy\n");
		SparseMat LyyS = L.block(state_IDm*dimPose, state_IDm*dimPose, state_diff*dimPose, state_diff*dimPose);
		solver.compute(LyyS);
		SparseMat IS(LyyS.rows(),LyyS.cols());
		for (int k=0; k < LyyS.rows(); k++){
			tripletListLyy.push_back(SparseTriplet(k,k,1));
		}
		IS.setFromTriplets(tripletListLyy.begin(), tripletListLyy.end());
		printf("Inverting Lyy\n");
		//MatrixXd LyyInv = Lyy.inverse();
		SparseMat LyySInv = solver.solve(IS);
		printf("Setting sparse\n");
		/*SparseMat LyySInv(LyyInv.rows(),LyyInv.cols());
		LyySInv.setBlock(0,0,LyyInv);*/
		ros::Duration d_init = ros::Time::now() - t_ros_init;
		printf("init OK...\n");

		ros::Time t_sparse = ros::Time::now();
		SparseMat LxxS = L.block(0,0,state_IDm*dimPose,state_IDm*dimPose);
		SparseMat LxyS = L.block(0, state_IDm*dimPose, state_IDm*dimPose, state_diff*dimPose);
		SparseMat LyxS = L.block(state_IDm*dimPose, 0, state_diff*dimPose, state_IDm*dimPose);

		SparseMat LzzS = L.block(state_IDn1*dimPose, state_IDn1*dimPose, L.rows()-state_IDn1*dimPose, L.cols()-state_IDn1*dimPose);
		SparseMat LyzS = L.block(state_IDm*dimPose, state_IDn1*dimPose, state_diff*dimPose, L.cols()-state_IDn1*dimPose);
		SparseMat LzyS = L.block(state_IDn1*dimPose, state_IDm*dimPose, L.rows()-state_IDn1*dimPose, state_diff*dimPose);
		SparseMat LxzS = L.block(0, state_IDn1*dimPose, state_IDm*dimPose, L.cols()-state_IDn1*dimPose);

		SparseMat LXS = LxxS - LxyS*LyySInv*LyxS;
		SparseMat RS = LxzS-LxyS*LyySInv*LyzS;
		SparseMat LZS = LzzS - LzyS*LyySInv*LyzS;
		printf("Sparse1 OK\n");

		for (int k=0; k < LXS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(LXS,k); it; ++it){
				double a = it.value();

				if(a!=0.0){
					tripletList.push_back(SparseTriplet(it.row(),it.col(),it.value()));
				}
			}
		}

		for (int k=0; k < LZS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(LZS,k); it; ++it){
				double a = it.value();
				if(a!=0.0){
					tripletList.push_back(SparseTriplet(it.row()+state_IDm*dimPose,it.col()+state_IDm*dimPose,it.value()));
				}
			}
		}
		for (int k=0; k < RS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(RS,k); it; ++it){
				double a = it.value();
				if(a!=0.0){
					tripletList.push_back(SparseTriplet(it.row(),(it.col()+state_IDm*dimPose),it.value()));
					tripletList.push_back(SparseTriplet((it.col()+state_IDm*dimPose),it.row(),it.value()));
				}
			}
		}

		L.resize(LXS.rows()+LZS.rows(),LXS.cols()+LZS.cols());
		L.setFromTriplets(tripletList.begin(), tripletList.end());
		ros::Duration d_sparse = ros::Time::now() - t_sparse;
		printf("L OK\n");

		ros::Time t_ros_end = ros::Time::now();
		VectorXd tmp_LXX = X.block(0,0,state_IDm*dimPose,1).eval();
		VectorXd tmp_LXZ = X.block(state_IDn1*dimPose,0,X.rows()-state_IDn1*dimPose,1).eval();

		X.resize(L.rows(),1);
		X.block(0,0,tmp_LXX.rows(),1) = tmp_LXX;
		X.block(tmp_LXX.rows(),0,tmp_LXZ.rows(),1) = tmp_LXZ;

		n.resize(L.rows(), 1);
		n = L*X;

		x.resize(2*dimPose,1);
		x = X.block(X.rows()-2*dimPose,0,2*dimPose,1);
		ros::Duration d_end = ros::Time::now() - t_ros_end;

		ros::Duration d_total = ros::Time::now()-t_ros_total;

		printf("Time recover: %f\n",d_recover.toSec());
		printf("Time init:    %f\n",d_init.toSec());
		printf("Time sparse:  %f\n",d_sparse.toSec());
		printf("Time end:     %f\n",d_end.toSec());
		printf("Time total:   %f\n",d_total.toSec());
		printf("Marginalizing state OK (%ld %d %ld)\n",n.rows()/dimPose,L.rows()/dimPose,X.rows()/dimPose);
	}else if(N>dimPose){
		printf("first state...\n");
		MatrixXd Lzz = L.block(dimPose, dimPose, dimPose, dimPose);
		MatrixXd Lyz = L.block(0, dimPose, dimPose, dimPose);
		MatrixXd Lzy = Lyz.transpose();
		MatrixXd Lyy = L.block(0,0, dimPose, dimPose);
		MatrixXd Lyy_inv = Lyy.inverse();
		L.resize(dimPose,dimPose);
		MatrixXd Omega_22 = Lzz - Lzy*Lyy_inv*Lyz;
		L.setBlock(0,0,Omega_22);

		X.resize(dimPose,1);
		X.block(0,0,dimPose,1) = xt;
		n.resize(dimPose, 1);
		n = L*X;
		x = xt;

	}else{ // this is the first state, no past states
		printf("Only one state, this is probably error...\n");
		x = xt;
		MatrixXd Pinv = Pxx.inverse();
		L.setBlock(0, 0, Pinv);
		n = L*xt;
	}
}

void LESDSF_class::MarginalizeO(int state_ID, bool no_sparse) {
	// augmentation with current state (xt, Pxx) and marginalization over the past state
	// in information form
	int N = n.rows();
	int Nx = x.rows();
	xt = SEMap.log(xtG);

	if(N>2*dimPose){
		ros::Time t_ros_total = ros::Time::now();
		std::vector<SparseTriplet> tripletList;

		printf("Marginalizing state %d (%ld %d %ld)\n",state_ID,n.rows()/dimPose,L.rows()/dimPose,X.rows()/dimPose);
		printf("sparsification: %d\n",!no_sparse);

		ros::Time t_ros_init = ros::Time::now();
		SparseMat LyySInv(dimPose,dimPose);
		MatrixXd Lyy = L.block(state_ID*dimPose, state_ID*dimPose, dimPose, dimPose);
		MatrixXd Lyy_inv = Lyy.inverse();
		LyySInv.setBlock(0,0,Lyy_inv);
		ros::Duration d_init = ros::Time::now() - t_ros_init;

		ros::Time t_sparse = ros::Time::now();
		SparseMat LxyS = L.block(0, state_ID*dimPose, state_ID*dimPose, dimPose);
		SparseMat LyxS = L.block(state_ID*dimPose, 0, dimPose, state_ID*dimPose);
		SparseMat LxxS = L.block(0,0,state_ID*dimPose,state_ID*dimPose);
		SparseMat LxzS = L.block(0,(state_ID+1)*dimPose,state_ID*dimPose,L.cols()-(state_ID+1)*dimPose);
		SparseMat LzzS = L.block((state_ID+1)*dimPose, (state_ID+1)*dimPose, L.rows()-(state_ID+1)*dimPose, L.cols()-(state_ID+1)*dimPose);
		SparseMat LyzS = L.block((state_ID)*dimPose, (state_ID+1)*dimPose, dimPose, L.cols()-(state_ID+1)*dimPose);
		SparseMat LzyS = L.block((state_ID+1)*dimPose, (state_ID)*dimPose, L.rows()-(state_ID+1)*dimPose, dimPose);

		SparseMat LXS = LxxS - LxyS*LyySInv*LyxS;
		SparseMat RS = LxzS-LxyS*LyySInv*LyzS;
		SparseMat LZS = LzzS - LzyS*LyySInv*LyzS;

		CLT_graph CLT_Lxx;
		MatrixXi G((LXS.rows()+LZS.rows())/dimPose,(LXS.cols()+LZS.cols())/dimPose);
		int before, after;
		before = after = 0;

		ros::Time t_sparsification = ros::Time::now();
		if(!no_sparse){

			G.setZero();

			FILE *pFile;
			char buffer[200];
			snprintf(buffer, sizeof(buffer), "/media/kruno/Data/TEST/SLAM/CLT/CLT_%d.txt",marg_id);
			marg_id++;
			pFile = fopen(buffer,"w");

			for (int k=0; k < LXS.outerSize(); ++k){
				for (Eigen::SparseMatrix<double>::InnerIterator it(LXS,k); it; ++it){
					double a = it.value();
					if(a!=0.0){
						int row_id = it.row();
						int col_id = it.col();
						row_id = row_id/dimPose;
						col_id = col_id/dimPose;
						if((col_id>row_id)){
							if(G(row_id,col_id)!=1){
								G(row_id,col_id)=1;
								before++;
								gain_ret weigth = InfGainLxx(row_id,col_id,LXS);
								CLT_Lxx.addEdge(row_id,col_id,weigth.Inf);
							}
						}
					}
				}
			}

			for (int k=0; k < RS.outerSize(); ++k){
				for (Eigen::SparseMatrix<double>::InnerIterator it(RS,k); it; ++it){
					double a = it.value();
					if(a!=0.0){
						int row_id = it.row();
						int col_id = it.col();
						before++;
						row_id = row_id/dimPose;
						col_id = col_id/dimPose;
						if(G(row_id,col_id+state_ID)!=1){
							G(row_id,col_id+state_ID)=1;
							gain_ret weigth = InfGainRS(row_id,col_id,LXS,RS,LZS);
							CLT_Lxx.addEdge(row_id,state_ID,weigth.Inf);
						}
					}
				}
			}

			for (int k=0; k < LZS.outerSize(); ++k){
				for (Eigen::SparseMatrix<double>::InnerIterator it(LZS,k); it; ++it){
					double a = it.value();
					if(a!=0.0){
						int row_id = it.row();
						int col_id = it.col();
						before++;
						row_id = row_id/dimPose;
						col_id = col_id/dimPose;
						if(G(row_id+state_ID,col_id+state_ID)!=1){
							G(row_id+state_ID,col_id+state_ID)=1;
							gain_ret weigth = InfGainLxx(row_id,col_id,LZS);
							CLT_Lxx.addEdge(row_id+state_ID,col_id+state_ID,weigth.Inf);
						}
					}
				}
			}

			CLT_Lxx.Init(LXS.rows()+LZS.rows());

			for(int k = 0; k < CLT_Lxx.edge.size(); k++){
				fprintf(pFile,"%d %d %f\n",CLT_Lxx.edge[k].src,CLT_Lxx.edge[k].dest,CLT_Lxx.edge[k].wt);
			}

			fprintf(pFile,"%d %d %d\n",-9999,-9999,-9999);

			CLT_Lxx.MST();

			G.setIdentity();

			for(int k = 0; k < CLT_Lxx.result.size(); k++){
				G(CLT_Lxx.result[k].src,CLT_Lxx.result[k].dest)=1;
				G(CLT_Lxx.result[k].dest,CLT_Lxx.result[k].src)=1;
				fprintf(pFile,"%d %d %f\n",CLT_Lxx.result[k].src,CLT_Lxx.result[k].dest,CLT_Lxx.result[k].wt);
			}
			fclose(pFile);
		}
		ros::Duration d_sparsification = ros::Time::now() - t_sparsification;

		for (int k=0; k < LXS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(LXS,k); it; ++it){
				double a = it.value();
				if(a!=0.0){
					int row_id = it.row();
					int col_id = it.col();
					int f_row_id = row_id/dimPose;
					int f_col_id = col_id/dimPose;
					if((G(f_row_id,f_col_id)==1) || no_sparse){
						tripletList.push_back(SparseTriplet(it.row(),it.col(),it.value()));
						after++;
					}
				}
			}
		}

		for (int k=0; k < RS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(RS,k); it; ++it){
				double a = it.value();
				if(a!=0.0){
					int row_id = it.row();
					int f_row_id = row_id/dimPose;
					if((G(f_row_id,state_ID)==1) || no_sparse){
						tripletList.push_back(SparseTriplet(it.row(),(it.col()+state_ID*dimPose),it.value()));
						tripletList.push_back(SparseTriplet((it.col()+state_ID*dimPose),it.row(),it.value()));
						after++;
					}
				}
			}
		}

		for (int k=0; k < LZS.outerSize(); ++k){
			for (Eigen::SparseMatrix<double>::InnerIterator it(LZS,k); it; ++it){
				double a = it.value();
				if(a!=0.0){
					int row_id = it.row();
					int col_id = it.col();
					int f_row_id = row_id/dimPose;
					int f_col_id = col_id/dimPose;
					if((G(f_row_id,f_col_id)==1) || no_sparse){
						tripletList.push_back(SparseTriplet(it.row()+state_ID*dimPose,it.col()+state_ID*dimPose,it.value()));
						after++;
					}
				}
			}
		}

		L.resize(LXS.rows()+LZS.rows(),LXS.cols()+LZS.cols());
		L.setFromTriplets(tripletList.begin(), tripletList.end());
		ros::Duration d_sparse = ros::Time::now() - t_sparse;

		ros::Time t_ros_end = ros::Time::now();
		VectorXd tmp_LXX = X.block(0,0,state_ID*dimPose,1).eval();
		VectorXd tmp_LXZ = X.block((state_ID+1)*dimPose,0,X.rows()-(state_ID+1)*dimPose,1).eval();

		X.resize(L.rows(),1);
		X.block(0,0,tmp_LXX.rows(),1) = tmp_LXX;
		X.block(tmp_LXX.rows(),0,tmp_LXZ.rows(),1) = tmp_LXZ;

		n.resize(L.rows(), 1);
		n = L*X;

		x.resize(2*dimPose,1);
		x = X.block(X.rows()-2*dimPose,0,2*dimPose,1);
		ros::Duration d_end = ros::Time::now() - t_ros_end;

		ros::Duration d_total = ros::Time::now()-t_ros_total;

		printf("Time init:           %f\n",d_init.toSec());
		printf("Time sparse:         %f\n",d_sparse.toSec());
		printf("Time sparsification: %f\n",d_sparsification.toSec());
		printf("Time end:            %f\n",d_end.toSec());
		printf("Time total:          %f\n",d_total.toSec());
		printf("b a: %d %d\n",before/dimPose/dimPose,after/dimPose/dimPose);
		printf("Marginalizing state OK (%ld %d %ld)\n",n.rows()/dimPose,L.rows()/dimPose,X.rows()/dimPose);

	}else if(N>dimPose){
		printf("first state...\n");
		MatrixXd Lzz = L.block(dimPose, dimPose, dimPose, dimPose);
		MatrixXd Lyz = L.block(0, dimPose, dimPose, dimPose);
		MatrixXd Lzy = Lyz.transpose();
		MatrixXd Lyy = L.block(0,0, dimPose, dimPose);
		MatrixXd Lyy_inv = Lyy.inverse();
		L.resize(dimPose,dimPose);
		MatrixXd Omega_22 = Lzz - Lzy*Lyy_inv*Lyz;
		L.setBlock(0,0,Omega_22);

		X.resize(dimPose,1);
		X.block(0,0,dimPose,1) = xt;
		n.resize(dimPose, 1);
		n = L*X;
		x = xt;

	}else{ // this is the first state, no past states
		printf("Only one state, this is probably error...\n");
		x = xt;
		MatrixXd Pinv = Pxx.inverse();
		L.setBlock(0, 0, Pinv);
		n = L*xt;
	}
}

void LESDSF_class::Predict() {
	// augmentation with current state (xt, Pxx) and marginalization over the past state
	// in information form
	xt = SEMap.log(xtG);
	if (x.rows() > dimPose) {
		MatrixXd Lxtxt_1 = L.block(L.rows()-dimPose, L.cols()-2*dimPose, dimPose, dimPose);
		MatrixXd Lxt_1xt_1 = L.block(L.rows()-2*dimPose, L.cols()-2*dimPose, dimPose, dimPose);
		MatrixXd Lxtxt = L.block(L.rows()-dimPose, L.cols()-dimPose, dimPose, dimPose);
		MatrixXd Lxt_1xt = L.block(L.rows()-2*dimPose, L.cols()-dimPose, dimPose, dimPose);

		MatrixXd Omega, Cross, Psi;
		MatrixXd Q_k_inv = Q.inverse();

		Omega = Lxtxt + F.transpose()* Q_k_inv * F;
		MatrixXd OmegaInv = Omega.inverse();
		Cross = Q_k_inv * F * OmegaInv * Lxtxt_1;
		Psi =(Q + F * Lxtxt.inverse() * F.transpose()).inverse();

		MatrixXd  LL = Lxt_1xt_1 - Lxt_1xt*OmegaInv*Lxtxt_1;
		//LL.print("LL\n");
		L.setBlock(L.rows()-2*dimPose, L.cols()-2*dimPose, LL);
		L.setBlock(L.rows()-dimPose, L.cols()-dimPose, Psi);
		L.setBlock(L.rows()-dimPose, L.cols()-2*dimPose, Cross);
		MatrixXd Crosst = Lxt_1xt*OmegaInv*F.transpose()*Q_k_inv;
		//Crosst.print("Crosst\n");
		L.setBlock(L.rows()-2*dimPose, L.cols()-dimPose, Crosst);

		VectorXd xt_1 = x.block(x.rows()-dimPose, 0, dimPose, 1);

		VectorXd ntz = n.block(n.rows()-dimPose, 0, dimPose, 1) - F.transpose()*Q_k_inv*(xt-F*xt_1);
		MatrixXd R = (Q_k_inv*F*OmegaInv*n.block(n.rows()-dimPose, 0, dimPose, 1)+Psi*(xt-F*xt_1)).eval();
		n.block(n.rows()-dimPose, 0, R.rows(), R.cols()) = R;
		R = n.block(n.rows()-2*dimPose, 0, dimPose, 1)-Lxtxt_1.transpose()*OmegaInv*ntz;
		n.block(n.rows()-2*dimPose, 0, R.rows(), R.cols()) = R;

		x.block(x.rows()-dimPose, 0, xt.rows(), xt.cols()) = xt;
	}
	else { // this is the first state, no past states
		x = xt;
		MatrixXd Pinv = Pxx.inverse();
		L.setBlock(0, 0, Pinv);
		n = L*xt;
	}
}

void LESDSF_class::Update(int id1, int id2, SE3Mat& zG, StateMat& R, vector<StateVector> &slam_trajectory){
	ros::Time t_total = ros::Time::now();
	//RecoverState();
	ros::Time t_init = ros::Time::now();

	StateMat invR = R.inverse();
	slam_trajectory.push_back(x.block(dimPose,0,dimPose,1));

	StateVector x1g = slam_trajectory[id1];
	//StateVector x1g = X.block(id1*dimPose,0,dimPose,1);
	StateVector x2g = slam_trajectory[id2];//X.block(id2*dimPose,0,dimPose,1);
	//StateVector x2g = X.block(id2*dimPose,0,dimPose,1);

	SE3Mat x1G = SEMap.exp(x1g);
	SE3Mat x2G = SEMap.exp(x2g);

	SE3Mat dT = x1G.inverse()*x2G;
	cout << "dT" << endl << dT << endl;
	cout << "zG" << endl << zG << endl;
	SE3Mat niG = (x1G.inverse()*x2G).inverse()*zG;
	StateVector nig = SEMap.log(niG);
	StateMat Hij;
	Hij = -SEMap.Ad(x2G.inverse())*SEMap.Ad(x1G);
	SparseMat L_;

	L_.resize(L.rows(),L.cols());
	L_=L;
	VectorXd n_(n.rows());
	L_.setBlock(id1*dimPose, id1*dimPose,StateMat(L.block(id1*dimPose,id1*dimPose,dimPose,dimPose))+Hij.transpose()*invR*Hij);
	L_.setBlock(id1*dimPose, id2*dimPose,StateMat(L.block(id1*dimPose,id2*dimPose,dimPose,dimPose))+Hij.transpose()*invR);
	L_.setBlock(id2*dimPose, id1*dimPose,StateMat(L.block(id2*dimPose,id1*dimPose,dimPose,dimPose))+invR*Hij);
	L_.setBlock(id2*dimPose, id2*dimPose,StateMat(L.block(id2*dimPose,id2*dimPose,dimPose,dimPose))+invR);

	n_.setZero();
	n_.block(id1*dimPose,0,dimPose,1) = Hij.transpose()*invR*nig;
	n_.block(id2*dimPose,0,dimPose,1) = invR*nig;
	VectorXd X_(n.rows());

	ros::Duration d_init = ros::Time::now() - t_init;

	ros::Time t_solve = ros::Time::now();

	solver.compute(L_);
	X_ = solver.solve(n_);

	ros::Duration d_solve = ros::Time::now() - t_solve;

	ros::Time t_sparse = ros::Time::now();

	SparseMat invPhiX_(n.rows(),n.rows());
	invPhiX_.reserve(VectorXi::Constant(n.rows(),dimPose));
	StateMat tmp_m,adjX_;
	SE3Mat expX, expX_, tmp_se3;
	StateVector d_T,tmp_v;

	X.resize(n.rows());
	for (int i = 0; i<n.rows(); i=i+dimPose){
		tmp_v = X_.block(i,0,dimPose,1);
		tmp_m = SEMap.PhiG(SEMap.adj(tmp_v));
		invPhiX_.setBlock(i,i,tmp_m.inverse());
		tmp_v = slam_trajectory[i/dimPose];
		//tmp_v = X.block(i,0,dimPose,1);
		expX = SEMap.exp(tmp_v);
		tmp_v = X_.block(i,0,dimPose,1);
		expX_ = SEMap.exp(tmp_v);
		tmp_se3 = expX*expX_;
		X.block(i,0,dimPose,1) = SEMap.log(tmp_se3);
	}

	ros::Duration d_sparse= ros::Time::now() - t_sparse;

	ros::Time t_end = ros::Time::now();

	L = invPhiX_.transpose()*L_*invPhiX_;
	n = L*X;

	tmp_v=X.block(X.rows()-dimPose,0,dimPose,1);
	xtG = SEMap.exp(tmp_v);
	x = X.block(X.rows()-2*dimPose,0,2*dimPose,1);

	ros::Duration d_end = ros::Time::now() - t_end;

	ros::Duration d_total = ros::Time::now() - t_total;
	t_glob_solve += d_total.toSec();
	total_updates++;
}

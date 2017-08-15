#define ESDSF_H
#define EIGEN_SUPERLU_SUPPORT 1

#define EIGEN_MATRIXBASE_PLUGIN "/home/kruno/catkin_ws/src/SLAM_git/cooperation_server/src/EigenMatrixBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "/home/kruno/catkin_ws/src/SLAM_git/cooperation_server/src/EigenSparseMatrixBaseAddons.h"
#include <fstream>
#include <iostream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <Eigen/Eigen> // whole Eigen matrix library (dense and sparse)
#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */

typedef Eigen::SparseMatrix<double> SparseMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
using namespace Eigen;
using namespace std;

struct self_tocka
{
	double x;
	double y;
};
struct L_mat_params{
	int rows, cols, numbers;
	int nsize;
};

int main ()
{
	using namespace boost::interprocess;
	//shared_memory_object::remove("shared_memory");
	self_tocka tmp_tck;
	L_mat_params L_P;
	Eigen::SimplicialCholesky<SparseMat, Eigen::Upper> solver;
	VectorXd X;
	VectorXd n;


	try{

		//Open already created shared memory object.
		shared_memory_object L_shm (open_only, "L_mat", read_only);
		shared_memory_object par_shm (open_only, "mat_params", read_only);
		shared_memory_object n_shm (open_only, "n_vec", read_only);


		//Map the whole shared memory in this process
		mapped_region L_region(L_shm, read_only);
		mapped_region par_region(par_shm, read_only);
		mapped_region n_region(n_shm, read_only);


		memcpy(&L_P,par_region.get_address(),sizeof(L_P));

		printf("params loaded: %d %d\n",L_P.nsize,L_P.cols);

		n.resize(L_P.nsize);
		memcpy(n.data(),n_region.get_address(),L_P.nsize*sizeof(double));

		std::vector<T> tripletList;
		T* list_t;
        list_t = (T*) malloc (L_P.numbers*sizeof(T));
		tripletList.reserve(L_P.numbers);
		memcpy(list_t,L_region.get_address(),L_P.numbers*sizeof(T));

		for (int i = 0; i < L_P.numbers; i++){
			tripletList.push_back(list_t[i]);
		}

		SparseMatrix<double> mat(L_P.rows,L_P.cols);
		mat.setFromTriplets(tripletList.begin(), tripletList.end());

		solver.compute(mat);
		X = solver.solve(n);
		X.print("n");

		std::cout << "Test successful!" << " " << L_P.nsize << std::endl;
	}
	catch(interprocess_exception &ex){
		std::cout << "Unexpected exception: " << ex.what() << std::endl;
		shared_memory_object::remove("shared_memory");
		return 1;
	}
	//shared_memory_object::remove("shared_memory");
	return 0;
}

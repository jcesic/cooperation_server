/* Eigen Custom Add-ons
	Written by Andrej Kitanov, Jan 2014
 */

/* neccesary to #include <fstream> before including Eigen header, i.e.
   #define EIGEN_SPARSEMATRIX_PLUGIN "EigenSparseMatrixBaseAddons.h"
   #include <fstream>
   #include <Eigen/Sparse>
 */
bool printToFile(const char *filename, char *msg /* = NULL */ ) const {
	std::ofstream file;

	file.open(filename);
	if (file.good()) {
		if (msg != 0)
			file << msg;

		file << *this;

		file.close();

	} else {
		printf("Unable to open file\n");
		return false;
	}

	return true;
}

void print(char *msg /* = NULL */ ) const {

	if (msg != 0)
		std::cout << msg;

	std::cout << *this << std::endl;

}

// set submatrix of a sparse matrix
//template<typename Derived>
inline void
setBlock(int idxRow, int idxCol, const Eigen::Ref<const Eigen::MatrixXd>& src)
{
	int height = src.rows();
	int width = src.cols();
	// check if submatrix is valid
	/*if ( idxRow+height > this->rows() || idxCol+width > this->cols() || idxRow < 0 || idxCol < 0 )
		throw std::string("Index of submatrix or submatrix out of matrix bounds!");*/

	for (int r_src = 0, r_dest = idxRow; r_src < height; r_src++, r_dest++){
		for (int c_src = 0, c_dest = idxCol; c_src < width; c_src++, c_dest++){
			this->coeffRef(r_dest, c_dest) = src(r_src, c_src);
		}
	}
	return;
}

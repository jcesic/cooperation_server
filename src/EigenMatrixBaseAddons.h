/* Eigen Custom Add-ons
	Written by Andrej Kitanov, Jan 2014
*/

/* neccesary to #include <fstream> before including Eigen header, i.e.
   #define EIGEN_MATRIXBASE_PLUGIN "EigenMatrixBaseAddons.h"
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

void setFromArray(const double* src) {

	for (int i = 0; i < this->rows(); i++)
		for (int j = 0; j < this->cols(); j++)
			this->coeffRef(i,j) = src[i*this->cols()+j];
	return;
}

void setFromArray(double** src) {

	for (int i = 0; i < this->rows(); i++)
		for (int j = 0; j < this->cols(); j++)
			this->coeffRef(i,j) = src[i][j];
	return;
}

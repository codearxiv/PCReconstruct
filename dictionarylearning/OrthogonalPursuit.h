//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef ORTHOGONALPURSUIT_H
#define ORTHOGONALPURSUIT_H


#include "constants.h"
//#include <vector>


class OrthogonalPursuit
{
	using Index = Eigen::Index;
	using MatrixXf = Eigen::MatrixXf;
	using VectorXf = Eigen::VectorXf;
	using MapVectf = Eigen::Map<VectorXf, ALIGNEDX>;
	using MapMtrxf = Eigen::Map<MatrixXf, ALIGNEDX>;
	template<typename T> using Map = Eigen::Map<T, ALIGNEDX>;
	template<typename T> using vector = std::vector<T, Eigen::aligned_allocator<T>>;

public:
	OrthogonalPursuit() :
		ndim(0), natm(0), lmax(0), dworkOffset(0),
		U(nullptr,0), V(nullptr,0), W(nullptr,0),
		XI(nullptr,0), I(nullptr,0),
		E(nullptr,0,0), F(nullptr,0,0),
		ETblk(nullptr,0,0), Fblk(nullptr,0,0),
		ldlt(nullptr), ldltSize(0) {}

	OrthogonalPursuit(const OrthogonalPursuit &op) : OrthogonalPursuit(){}
	//{ ensure(sa.ndim, sa.natm, sa.lmax); }

	~OrthogonalPursuit(){
		delete ldlt;
    }

	void ensure(Index nd, Index na, Index lm);

	void operator() (
			const VectorXf& Y, const MatrixXf& D, Index l,
			VectorXf& X, VectorXf& R);

private:
	void ensureWorkspace();

	Index ndim; // Size of signal vector.
	Index natm; // No. of 'atoms' in dictionary.
	Index lmax; // maximum sparsity constraint <= natm.

	vector<float> dwork;
	size_t dworkOffset;
	vector<Index> iwork;
	// Work-space mapped onto work buffer.
	MapVectf U, V, W, XI;
	Map< Eigen::Matrix<Index,Eigen::Dynamic,1> > I;
	MapMtrxf E, F, ETblk, Fblk;

	Eigen::LDLT<MatrixXf> *ldlt;
	Index ldltSize;


};

#endif // ORTHOGONALPURSUIT_H


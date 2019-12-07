#include "CoverTreePoint.h"
#include "pt_to_pt_distsq.h"
//#include <iostream>
//#include <Eigen/Dense>

using Eigen::VectorXf;


double CoverTreePoint::distance(const CoverTreePoint& p) const {
	const VectorXf& vec2 = p.getVec();
	assert(vec2.size() == _vec.size());
	return sqrt(pt_to_pt_distsq(_vec, vec2));
}

const VectorXf& CoverTreePoint::getVec() const {
    return _vec;
}

int CoverTreePoint::getId() const {
	return _id;
}

void CoverTreePoint::set(const VectorXf& v, int id) {
	_vec = v;
	_id = id;
}

void CoverTreePoint::print() const {
	std::cout << "point " << _id << ": " << _vec << "\n";
}

bool CoverTreePoint::operator==(const CoverTreePoint& p) const {
	return (_id==p.getId());
}

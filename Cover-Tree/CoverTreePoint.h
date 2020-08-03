//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

//
//  Brief description
//     Point on a cover tree stored as a vector in Eigen.

#ifndef _COVERTREEPOINT_H
#define _COVERTREEPOINT_H

#include "pt_to_pt_distsq.h"
//#include <Eigen/Dense>


template<class T>
class CoverTreePoint {
private:
	T _vec;
	size_t _id;
public:
	CoverTreePoint(T v, size_t id) : _vec(v), _id(id) {}
	double distance(const CoverTreePoint<T>& p) const;
	const T& getVec() const;
	size_t getId() const;
	void set(const T& v, size_t id);
    void print() const;
	bool operator==(const CoverTreePoint<T>&) const;
};

template<class T>
double CoverTreePoint<T>::distance(const CoverTreePoint<T>& p) const {
	const T& vec2 = p.getVec();
	assert(vec2.size() == _vec.size());
	double distsq = 0.0;
	for(size_t i=0; i < _vec.size(); ++i) {
		distsq += (_vec[i]-vec2[i])*(_vec[i]-vec2[i]);
	}
	return distsq;
}

template<class T>
const T& CoverTreePoint<T>::getVec() const {
	return _vec;
}

template<class T>
size_t CoverTreePoint<T>::getId() const {
	return _id;
}

template<class T>
void CoverTreePoint<T>::set(const T& v, size_t id) {
	_vec = v;
	_id = id;
}

template<class T>
void CoverTreePoint<T>::print() const {
	std::cout << "point " << _id << ": " << _vec << "\n";
}

template<class T>
bool CoverTreePoint<T>::operator==(const CoverTreePoint<T>& p) const {
	return (_id==p.getId());
}


#endif // _COVERTREEPOINT_H




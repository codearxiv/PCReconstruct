//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//
//  Brief description
//     Point on a cover tree stored as a vector in Eigen.

#ifndef _UNIT_SPHERE_POINT_H
#define _UNIT_SPHERE_POINT_H

#include "pt_to_pt_distsq.h"
//#include <Eigen/Dense>


template<class T>
class CoverTreePoint {
private:
	T _vec;
	int _id;
public:
	CoverTreePoint(T v, int id) : _vec(v), _id(id) {}
	double distance(const CoverTreePoint<T>& p) const;
	const T& getVec() const;
	int getId() const;
	void set(const T& v, int id);
    void print() const;
	bool operator==(const CoverTreePoint<T>&) const;
};

template<class T>
double CoverTreePoint<T>::distance(const CoverTreePoint<T>& p) const {
	const T& vec2 = p.getVec();
	assert(vec2.size() == _vec.size());
	return sqrt(pt_to_pt_distsq<T>(_vec, vec2));
}

template<class T>
const T& CoverTreePoint<T>::getVec() const {
	return _vec;
}

template<class T>
int CoverTreePoint<T>::getId() const {
	return _id;
}

template<class T>
void CoverTreePoint<T>::set(const T& v, int id) {
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


#endif // _UNIT_SPHERE_POINT_H




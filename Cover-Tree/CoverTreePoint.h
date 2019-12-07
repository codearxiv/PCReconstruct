//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//
//  Brief description
//     Point on a cover tree stored as a vector in Eigen.

#ifndef _UNIT_SPHERE_POINT_H
#define _UNIT_SPHERE_POINT_H

//#include <Eigen/Dense>



class CoverTreePoint
{
private:
	Eigen::VectorXf _vec;
	int _id;
public:
	CoverTreePoint(Eigen::VectorXf v, int id) : _vec(v), _id(id) {}
	double distance(const CoverTreePoint& p) const;
	const Eigen::VectorXf& getVec() const;
	int getId() const;
	void set(const Eigen::VectorXf& v, int id);
    void print() const;
	bool operator==(const CoverTreePoint&) const;
};

#endif // _UNIT_SPHERE_POINT_H


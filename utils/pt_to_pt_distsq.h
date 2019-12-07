//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef PT_TO_PT_DISTSQ_H
#define PT_TO_PT_DISTSQ_H

template<class T>
double pt_to_pt_distsq(const T& v, const T& w);

template<class T>
double pt_to_pt_distsq(const T& v, const T& w)
{
	double distsq = 0.0;
	for(size_t i=0; i < v.size(); ++i) {
		distsq += (v[i]-w[i])*(v[i]-w[i]);
	}
	return distsq;
}



#endif // PT_TO_PT_DISTSQ_H

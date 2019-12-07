//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_NORMAL_H
#define CLOUD_NORMAL_H

Vector3f cloud_normal(Vector3f p0, const Matrix3Xf& cloud, int niters, double zeroTol = 0.0f);

#endif // CLOUD_NORMAL_H

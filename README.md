# PCReconstruct

A point cloud reconstruction tool written in Qt C++ and OpenMP.

Takes a PCL point cloud surface and fills in gaps or densifies sparse regions by learning from the various surface features
of the cloud. This is done using a "continuous signal" variation of the k-SVD dictionary learning algorithm adapted to unstructured point clouds.

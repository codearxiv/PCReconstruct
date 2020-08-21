# PCReconstruct

A point cloud reconstruction tool written in Qt C++, OpenMP, and OpenGL.

Takes a PCL point cloud surface and fills in gaps or densifies sparse regions by learning from the various surface features of the cloud.

This is done using a "continuous signal" variation of the k-SVD dictionary learning algorithm adapted to unstructured point clouds.

Example:
Original surface sampled with 35000 points: capture5a

Sparsified down to 1225 points: capture5b

Increasing the default densification field to 1.5 to get a denser reconstruction: capture5c

This gives this reconstruction of the sparsified point cloud: capture5d

Another Example:
Original surface sampled with 35000 points: capture6a

Heavily decimated with random holes: capture6b

Increasing the default patch size field to 500 to take into the large gaps relative to sampling density: capture6c

This gives the follwing reconstruction of the decimated point cloud: capture6d

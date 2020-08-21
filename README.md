# PCReconstruct

A point cloud reconstruction tool written in Qt C++, OpenMP, and OpenGL.

Takes a PCL point cloud surface and fills in gaps or densifies sparse regions by learning from the various surface features of the cloud.

This is done using a "continuous signal" variation of the k-SVD dictionary learning algorithm adapted to unstructured point clouds.

## Example:
Original surface sampled with 35000 points: 

![1](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5a.PNG)

Sparsified down to 1225 points: 

![2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5b.PNG)

Increasing the default densification field to 1.5 to get a denser reconstruction:

![3](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5c.PNG)

This gives this reconstruction of the sparsified point cloud: 

![4](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5d.PNG)


## Another Example:
Original surface sampled with 35000 points:

![5](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture6a.PNG)

Heavily decimated with random holes:

![6](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture6b.PNG)

Increasing the default patch size field to 500 to take into account the large gaps relative to sampling density:

![7](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture6c.PNG)

This gives the follwing reconstruction of the decimated point cloud:

![8](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture6d.PNG)


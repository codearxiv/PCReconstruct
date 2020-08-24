# PCReconstruct

A point cloud reconstruction tool written in Qt C++, using Eigen, OpenMP, and OpenGL.

Takes a PCL point cloud surface and fills in gaps or densifies sparse regions by learning from the various surface features of the cloud.

This is done using a "continuous dictionary" variation of the k-SVD dictionary learning algorithm adapted to unstructured point clouds.

## TODO: 
* Still struggles in areas of very high curvature. Smaller patch sizes may have to be selected here automatically. 
* The slow signal setup step of reconstruction could be parallelized.

## Sparsified Stanford Bunny:
Original Stanford bunny sampled with 35847 points: 

![1a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1a.PNG)

Sparsifying the mid section to 5% after shrinking the bounding box to exclude
the ears (there are not enough points around the ear tips to approximate normals 
after sparsifying).

![1b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1b.PNG)
![1c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1c.PNG)

Increasing the default densification field to 1.5 to get a denser reconstruction,
and making sure the "use points outside bounding box" field is "True" to get better
agreement around the boundary of the bounding box:

![1d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1d.PNG)

The resulting reconstruction of the sparsified bunny: 

![1e](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1e.PNG)

## Decimated Armadillo:
Original armadillo sampled with 172,974 points: 

![2a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2a.PNG)
![2aa](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2aa.PNG)

Decimating it's mid body with random holes:

![2b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2b.PNG)
![2bb](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2bb.PNG)

Increasing the default patch size field to 300 to take into account the large gaps relative to sampling density:

![2c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2c.PNG)

The resulting reconstruction of the decimated armadillo (reconstructed points highlighted and unhighlighted): 

![2d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2d.PNG)
![2dd](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2dd.PNG)
![2e](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2e.PNG)
![2ee](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2ee.PNG)


## Sparsified Random Surface:
Original surface sampled with 35000 points: 

![5a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5a.PNG)

Sparsified down to 1225 points: 

![5b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5b.PNG)

Increasing the default densification field to 1.5 to get a denser reconstruction:

![5c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5c.PNG)

The resulting reconstruction of the sparsified point cloud: 

![5d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture5d.PNG)


## Decimated Random Surface:
Original surface sampled with 15000 points:

![7a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture7a.PNG)

Heavily decimated with random holes:

![7b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture7b.PNG)

Increasing the default patch size field to 150 to take into account the large gaps relative to sampling density:

![7c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture7c.PNG)

The resulting reconstruction of the decimated point cloud:

![7d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture7d.PNG)


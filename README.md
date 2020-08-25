# PCReconstruct

A point cloud reconstruction tool written in Qt C++, using Eigen, OpenMP, and OpenGL.

Takes a PCL point cloud surface and fills in gaps or densifies sparse regions by learning from the various surface features of the cloud.

This is done using a "continuous dictionary" variation of the k-SVD dictionary learning algorithm adapted to unstructured point clouds.

## TODO: 
* Reconstruction struggles in areas of very high curvature, producing artifacts that go off in random directions. 
* The slow signal setup step of reconstruction could be parallelized.
* Take point colour values into account.

## Sparsified Stanford Bunny:
Original Stanford bunny sampled with 35847 points: 

![1a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1a.PNG)

Sparsifying the mid section to 4% after shrinking the bounding box to exclude
the ears (there are not enough points around the ear tips to approximate normals 
after sparsifying, and the curvature there is too high as well).

![1b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1b.PNG)
![1c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1c.PNG)

We increase the default densification field to 1.5 to get a denser reconstruction,
and set patch frequency to 2 since we dont expect much bumpiness in the final result
within a patch size of 50. We also want tthe "use points outside bounding box" 
field set "True" to get better agreement around the boundary of the bounding box:

![1d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1d.PNG)

The resulting reconstruction of the sparsified bunny: 

![1e](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture1e.PNG)

## Decimated Armadillo:
Original armadillo sampled with 172,974 points: 

![2a](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2a.PNG)
![2a2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2a2.PNG)

Decimating it's mid body with random holes:

![2b](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2b.PNG)
![2b2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2b2.PNG)

We increase the default patch size field to 300 to take into account the large gaps relative 
to sampling density:

![2c](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2c.PNG)

The resulting reconstruction of the decimated armadillo, with reconstructed points highlighted: 

![2d](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2d.PNG)
![2d2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2d2.PNG)

When we unhighlight the new points the back looks unnaturally smooth

![2e](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2e.PNG)

But this is because of the normals need updating using a smaller patch size

![2e2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2e2.PNG)

Doing this we get

![2f](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2f.PNG)
![2f2](https://github.com/codearxiv/PCReconstruct/blob/master/images/Capture2f2.PNG)


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


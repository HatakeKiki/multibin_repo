# Implementation of Multibin in C++ with frugally-deep

​	Paper: 3D Bounding Box Estimation Using Deep Learning and Geometry

​	The main aim of this project is to predict the dimensions and heading of the vehicle in 3D coordinate with C++.  So the net model will be constructed and trained with keras for its convenience and transferred into .json so it can be used with C++.

​	The regressed parameters are still not accurate enough and slight differences may cause severe deviation in 3D coordinate. I intend to combine with sparse Lidar point cloud to rectify the deviation in my own project.

## Prerequisites
1. frugally-deep
2. opencv
3. python
4. keras

## Usage
1. Git clone this project: https://github.com/jqfromsjtu/multibin_repo.git

2. Git clone model: https://github.com/cersar/3D_detection.git

     Several modifications need to be done because the difference in keras.

     Remove lambda layer and normalize orientation output in loss function because frugally-deep doesn't support lambda.

     You may unfreeze last few layers (layer.trainable = True) to get more accurate output.

3. Download KITTI dataset and change the corresponding path in **train.py** and **utils.hpp**

4.  Construct and train the net. Save the architecture and weights to a  HDF5 file

5.  Follow the instruction to setup frugally-deep environment and generate JSON file

     https://github.com/Dobiasd/frugally-deep#usage

 5. Copy the following files into /multibin_repo/include

    /frugally-deep/include/fdeep

    /eigen/Eigen

    /FunctionalPlus/include/fplus

    /json/include/nlomann

    Then multibin_repo/include should have **fdeep, Eigen, fplus and nlomann**.

6. Replace **multibin_repro/include/fdeep/tensor.hpp**  and **multibin_repro/include/fplus/container_common.hpp** with the two HPP files in include_replace
7.  Move the JSON file into multibin_repro/model/

​	This is because the input image needs to subtract a fixed value in every channel and I haven't figure out a way to accomplish this pre-procession with the API provided by frugally-deep or opencv. So I modified several functions in tensor.hpp and container_common.hpp.

8. compile and run the project

```shell
cd ~/multibin_reproduction
cd multibin_repro
mkdir -p build && cd build
cmake ..
make
./multibin_repro
```

## Reference

https://github.com/cersar/3D_detection

https://github.com/experiencor/image-to-3d-bbox

https://github.com/shashwat14/Multibin

https://github.com/smallcorgi/3D-Deepbox

https://github.com/experiencor/didi-starter

https://github.com/Dobiasd/frugally-deep

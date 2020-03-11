# 3D regitration of point clouds grabbed via Kinect

## Dependencies
 - pcl 1.7

## Build
 - `git clone https://github.com/DanieleVeri/pcl_registration`
 - `mkdir pcl_registration/build`
 - `cd pcl_registration/build`
 - `cmake ..`
 - `make`

## Run
 - `./capture <folder>` launch the point cloud grabber and store .pcd in `<folder>`
 - `./generate_model <folder>` launch the model generator, with .pcd in `<folder>` as input

## Algorithm overview
Parameter tuning: `hyperparameter.json`
#### 1. Depth images acquired are preprocessed:
 - z filtered
 - downsampeld
 - main plain removal
 - outlier removal
 - keypoint extraction
 - sift descriptor extraction FPFH
#### 2. Pairwise registration 
 - many `SampleConsensusInitialAlignment` computed
 - clustering of computed 4x4 transformation matrices
 - main centroid used as init alignement
 - ICP refinement
 - main objects extracted from the pointclouds
 - then aligned according the computed transformation
#### 3. Cloud smoothing and mesh generation
 - output smoothed point cloud
 - output vtk mesh

## TODO
 - multithread registration
 - backtrack along global alignement
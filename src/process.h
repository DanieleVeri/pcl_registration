#ifndef PROCESS_H
#define PROCESS_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>

using namespace pcl;
using namespace std;

typedef PointCloud<PointXYZRGB>::Ptr Cloud;
typedef PointCloud<FPFHSignature33>::Ptr Descriptor;

typedef struct {
    float max_z_depth = 1.5;
    float downsample_grid = 0.005;
    int   outlier_meank = 100;
    float dominant_plane_thr = 0.02;
    int   clustering_min_perc = 30;
    float clustering_tollerance = 0.05;
    float keypoint_min_scale = 0.002;
    int   keypoint_nr_octaves = 4;
    int   keypoint_nr_scales_per_octave = 5;
    float keypoint_min_contrast = 1;
    float keypoint_radius = 0.01;
    float normal_radius = 0.05;
    float descriptor_radius = 0.1;
} ProcessParam;

const ProcessParam DEFAULT;

typedef struct {
    int no;
    Cloud cloud;
    Cloud keypoints;
    Descriptor descriptor;
} Processed;

Processed process(int no, Cloud cloud, ProcessParam param);

Processed update(int no, Cloud cloud, ProcessParam param);

#endif
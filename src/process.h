#ifndef PROCESS_H
#define PROCESS_H

#include <iostream>
#include "json.hpp"
#include <fstream>
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
using json = nlohmann::json;

typedef PointCloud<PointXYZRGB>::Ptr Cloud;
typedef PointCloud<FPFHSignature33>::Ptr Descriptor;

typedef struct {
    float max_z_depth;
    float downsample_grid;
    int   outlier_meank;
    float dominant_plane_thr;
    int   clustering_min_perc;
    float clustering_tollerance;
    float keypoint_min_scale;
    int   keypoint_nr_octaves;
    int   keypoint_nr_scales_per_octave;
    float keypoint_min_contrast;
    float keypoint_radius;
    float normal_radius;
    float descriptor_radius;
} ProcessParam;

typedef struct {
    int no;
    Cloud cloud;
    Cloud keypoints;
    Descriptor descriptor;
} Processed;

ProcessParam load_process_param(const char *path);
Processed process(int no, Cloud cloud, ProcessParam param);
Cloud extract_main_cluster(Cloud in, ProcessParam param);

#endif
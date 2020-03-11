#ifndef MESH_H
#define MESH_H

#include "json.hpp"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

using namespace pcl;
using namespace std;
using json = nlohmann::json;

typedef struct {
    float mls_radius;
    int   mls_order;
    int   normal_radius;
    float gp3_radius;
    float gp3_mu;
    int   gp3_max_nn;
} MeshParam;

MeshParam load_mesh_param(const char *path);

PointCloud<PointNormal>::Ptr smooth_cloud(PointCloud<PointXYZRGB>::Ptr cloud, MeshParam param);

int save_vtk(PointCloud<PointNormal>::Ptr cloud, const char *path, MeshParam param);

#endif
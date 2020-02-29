#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "process.h"
#include <thread>
#include <mutex>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/correspondence_estimation.h>

using namespace std;
using namespace pcl;

typedef struct {
	float ia_min_sample_distance = 0.05;
	float ia_max_correspondence_distance = 0.15;
	int   ia_iterations = 1000;

	float icp_max_correspondence_distance = 0.15;
	float icp_outlier_rejection_threshold = 0.05;
	float icp_transformation_eps = 1e-9;
	int   icp_max_iterations = 50;
	float icp_euclidean_fitness_eps = 1;
} RegistrationParam;

Processed register_pair(Processed src, Processed dst, RegistrationParam param, Eigen::Matrix4f* trans, boost::shared_ptr<visualization::CloudViewer> viewer);

Processed register_r(vector<Processed> clouds, boost::shared_ptr<visualization::CloudViewer> viewer);

#endif
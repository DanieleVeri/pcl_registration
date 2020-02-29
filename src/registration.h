#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "process.h"
#include "json.hpp"
#include <fstream>
#include <thread>
#include <mutex>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/correspondence_estimation.h>

using namespace std;
using namespace pcl;
using json = nlohmann::json;

typedef struct {
	float ia_min_sample_distance;
	float ia_max_correspondence_distance;
	int   ia_iterations;

	float icp_max_correspondence_distance;
	float icp_outlier_rejection_threshold;
	float icp_transformation_eps;
	int   icp_max_iterations;
	float icp_euclidean_fitness_eps;
} RegistrationParam;

RegistrationParam load_registration_param(const char *path);
Processed register_pair(Processed src, Processed dst, RegistrationParam param, Eigen::Matrix4f* trans, boost::shared_ptr<visualization::CloudViewer> viewer);
Processed register_r(vector<Processed> clouds, RegistrationParam param, boost::shared_ptr<visualization::CloudViewer> viewer);

#endif
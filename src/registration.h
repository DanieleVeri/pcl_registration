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
	int   init_samples;
	int   dbscan_min;
	float dbscan_eps;

	float ia_min_sample_distance;
	float ia_max_correspondence_distance;
	float ia_outlier_rejection_threshold;
	float ia_transformation_eps;
	float ia_euclidean_fitness_eps;
	int   ia_iterations;

	float icp_max_correspondence_distance;
	float icp_outlier_rejection_threshold;
	float icp_transformation_eps;
	int   icp_max_iterations;
	float icp_euclidean_fitness_eps;
} RegistrationParam;

RegistrationParam load_registration_param(const char *path);

Eigen::Matrix4f register_pair(Processed src, Processed dst, 
						ProcessParam process_param, RegistrationParam param, 
						boost::shared_ptr<visualization::CloudViewer> viewer);

Cloud register_r(vector<Processed> clouds, 
						ProcessParam process_param, RegistrationParam reg_param, 
						boost::shared_ptr<visualization::CloudViewer> viewer);

// Matrix utils

Eigen::Matrix4f vec2matrix(vector<float> v);

vector<float> matrix2vec(Eigen::Matrix4f m);

void print_matrix(Eigen::Matrix4f m);

#endif
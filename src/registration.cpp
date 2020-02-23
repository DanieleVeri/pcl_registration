#include "process.h"
#include <thread>
#include <mutex>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

using namespace std;
using namespace pcl;

Processed register_pair(Processed src, Processed dst) {
	// registration - initial alignment
	float min_sample_distance = 0.025;
	float max_correspondence_distance = 0.01;
	int nr_iterations = 500;

	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();
	SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
	sac_ia.setMinSampleDistance(min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
	sac_ia.setMaximumIterations(nr_iterations);
	sac_ia.setInputCloud(src.keypoints);
	sac_ia.setSourceFeatures(src.descriptor);
	sac_ia.setInputTarget(dst.keypoints);
	sac_ia.setTargetFeatures(dst.descriptor);
	PointCloud<PointXYZRGB>::Ptr aligned_source(new PointCloud<PointXYZRGB>);
	sac_ia.align(*aligned_source);
	initial_alignment = sac_ia.getFinalTransformation();
	console::print_info("computed initial alignment\n");

	PointCloud<PointXYZRGB>::Ptr init_aligned_source(new PointCloud<PointXYZRGB>);
	transformPointCloud (*src.cloud, *init_aligned_source, initial_alignment);
	*init_aligned_source += *dst.cloud;
	//viewer->showCloud(init_aligned_source);
	
	// ICP alignment
	float max_correspondence_distance_refine = 0.05;
	float outlier_rejection_threshold = 0.01;
	float transformation_eps = 1e-8;
	int max_iterations = 50;
	float euclidean_fitness_eps = 1;

	Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity();
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setMaxCorrespondenceDistance(max_correspondence_distance_refine);
	icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
	icp.setTransformationEpsilon(transformation_eps);
	icp.setMaximumIterations(max_iterations);
	icp.setEuclideanFitnessEpsilon(euclidean_fitness_eps);
	////////////////////////////////////////icp.setCorrespondenceEstimation
	icp.setInputCloud(src.cloud);
	icp.setInputTarget(dst.cloud);
	PointCloud<PointXYZRGB> registration_output;
	icp.align(registration_output);
	final_alignment = icp.getFinalTransformation() * initial_alignment;

	Cloud c (new PointCloud<PointXYZRGB>);
	transformPointCloud(*src.cloud, *c, final_alignment);
	console::print_info("refined alignment, score: %f\n", icp.getFitnessScore());
	(*c) += (*dst.cloud);

	Processed p = update(dst.no, c, DEFAULT);
	return p;
}

Processed register_r(vector<Processed> clouds, boost::shared_ptr<visualization::CloudViewer> viewer) 
{
	if (clouds.size() == 0)
		throw BadArgumentException("empty cloud list");

	if (clouds.size() == 1)
		return clouds[0];

	vector<Processed> next;
	for (size_t i = 0; i < clouds.size()-1; i+=2) {
		PCL_INFO("============== %d %d / %d ", clouds[i].no, clouds[i+1].no, clouds.size());
		auto merged = register_pair(clouds[i], clouds[i+1]);
		next.push_back(merged);
		viewer->showCloud(merged.cloud);
		char c[22];
		cin>>c;
	}
	if (clouds.size() % 2 == 1)
		next.push_back(clouds.back());

	return register_r(next, viewer);
}

int main(int argc, char **argv)
{
	boost::shared_ptr<visualization::CloudViewer> viewer(new visualization::CloudViewer("3D Viewer"));

	// read data dir
	DIR *dir;
	struct dirent *ent;
	string dir_path = "../data/";
	if ((dir = opendir(dir_path.c_str())) == NULL)
	{
		perror("");
		return EXIT_FAILURE;
	}
	vector<string> files = vector<string>();
	while ((ent = readdir(dir)) != NULL)
	{
		if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
			continue;
		files.push_back(ent->d_name);
	}
	sort(files.begin(), files.end());

	// load all point clouds
	vector<thread> threads;
	vector<Processed> clouds;
	mutex clouds_mutex;
	for (size_t i = 0; i < files.size(); ++i)
	{
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		// loading 
		string path = dir_path + files[i];
		io::loadPCDFile(path, *cloud);
		cout << i <<" -> loaded " << cloud->width * cloud->height << " data points" << endl;
		// process 
		threads.push_back(thread([i, cloud, &clouds, &clouds_mutex, viewer]() mutable {
			auto processed = process(i, cloud, DEFAULT);
			lock_guard<mutex> lock(clouds_mutex, adopt_lock);
			viewer->showCloud(processed.cloud);
			clouds.push_back(processed);
		}));
	}
	closedir(dir);
	for (auto& th : threads) 
    	th.join();

	sort(clouds.begin(), clouds.end(), 
		[](const Processed & a, const Processed & b) -> bool {return a.no < b.no;});

	auto final = register_r(clouds, viewer);

	viewer->showCloud(final.cloud);

	while (1)
		sleep(1);
	return (0);
}

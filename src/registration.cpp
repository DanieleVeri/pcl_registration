#include "registration.h"

#include <fstream>
#define WUI {PCL_INFO(">>> Input to continue <<<"); string s; cin >>s;}

Processed register_pair(Processed src, Processed dst, RegistrationParam param, Eigen::Matrix4f* trans, boost::shared_ptr<visualization::CloudViewer> viewer) {
	int iterations = 0;
	while(1) {
		// initial alignment
		Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();
		SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
		sac_ia.setMinSampleDistance(param.ia_min_sample_distance);
		sac_ia.setMaxCorrespondenceDistance(param.ia_max_correspondence_distance);
		sac_ia.setMaximumIterations(param.ia_iterations);
		sac_ia.setInputSource(src.keypoints);
		sac_ia.setSourceFeatures(src.descriptor);
		sac_ia.setInputTarget(dst.keypoints);
		sac_ia.setTargetFeatures(dst.descriptor);
		PointCloud<PointXYZRGB>::Ptr aligned_source(new PointCloud<PointXYZRGB>);
		sac_ia.align(*aligned_source);
		initial_alignment = sac_ia.getFinalTransformation();
		PCL_INFO("[%d - %d] -> computed initial alignment score: %f\n", src.no, dst.no, sac_ia.getFitnessScore());

		if (iterations > 0 && *trans != Eigen::Matrix4f::Identity()) {
			initial_alignment = *trans;
		}

		PointCloud<PointXYZRGB>::Ptr init_aligned_source(new PointCloud<PointXYZRGB>);
		transformPointCloud (*src.cloud, *init_aligned_source, initial_alignment);
		*init_aligned_source += *dst.cloud;
		viewer->showCloud(init_aligned_source);
		WUI
	
		// ICP alignment
		Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity();
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		search::KdTree<PointXYZRGB>::Ptr kd_tree(new search::KdTree<PointXYZRGB>);
		icp.setSearchMethodTarget(kd_tree);
		icp.setMaxCorrespondenceDistance(param.icp_max_correspondence_distance);
		icp.setRANSACOutlierRejectionThreshold(param.icp_outlier_rejection_threshold);
		icp.setTransformationEpsilon(param.icp_transformation_eps);
		icp.setMaximumIterations(param.icp_max_iterations);
		icp.setEuclideanFitnessEpsilon(param.icp_euclidean_fitness_eps);
		icp.setInputSource(init_aligned_source);
		icp.setInputTarget(dst.cloud);
		PointCloud<PointXYZRGB> registration_output;
		icp.align(registration_output);
		final_alignment = icp.getFinalTransformation() * initial_alignment;
		stringstream ss;
		ss << final_alignment;
		auto a = final_alignment.block<3,3>(0,0).eulerAngles(2, 1, 0);
		float yaw = a(0, 0);
		float pitch = a(1, 0);
		float roll = a(2, 0);

		Cloud c (new PointCloud<PointXYZRGB>);
		transformPointCloud(*src.cloud, *c, final_alignment);
		PCL_INFO("[%d - %d] -> refined alignment, covergence: %d, score: %f, matrix: \n%s\n",
			src.no, dst.no, icp.hasConverged(), icp.getFitnessScore(), ss.str().c_str());
		PCL_INFO("roll: %f    pitch: %f    yaw: %f\n", roll, pitch, yaw);
		(*c) += (*dst.cloud);
		/* smooth
		search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
		PointCloud<PointNormal> mls_points;
		MovingLeastSquares<PointXYZRGB, PointNormal> mls;
		mls.setComputeNormals (true);
		mls.setInputCloud(c);
		mls.setPolynomialOrder(2);
		mls.setSearchMethod (tree);
		mls.setSearchRadius (0.1);
		mls.process (mls_points);
		copyPointCloud(mls_points, *c);
		*/
		viewer->showCloud(c);
		int stappo;
		cout<<"stappo?";
		cin >>stappo;
		if(!stappo) {
			ifstream file;
			file.open("params.txt");
			file >> param.ia_min_sample_distance;
			cout <<"param.ia_min_sample_distance: "<<param.ia_min_sample_distance<<endl;
			file >> param.ia_max_correspondence_distance;
			cout <<"param.ia_max_correspondence_distance: "<<param.ia_max_correspondence_distance<<endl;
			file >> param.ia_iterations;
			cout <<"param.ia_iterations: "<<param.ia_iterations<<endl;
			file >> param.icp_max_correspondence_distance;
			cout <<"param.icp_max_correspondence_distance: "<<param.icp_max_correspondence_distance<<endl;
			file >> param.icp_outlier_rejection_threshold;
			cout <<"param.icp_outlier_rejection_threshold: "<<param.icp_outlier_rejection_threshold<<endl;
			file >> param.icp_transformation_eps;
			cout <<"param.icp_transformation_eps: "<<param.icp_transformation_eps<<endl;
			file >> param.icp_euclidean_fitness_eps;
			cout <<"param.icp_euclidean_fitness_eps: "<<param.icp_euclidean_fitness_eps<<endl;
			file >> param.icp_max_iterations;
			cout <<"param.icp_max_iterations: "<<param.icp_max_iterations<<endl;
			file.close();
			iterations++;
		} else {
			*trans = final_alignment;
			break;
		}
	}

	Processed p;// = update(dst.no, c, DEFAULT);
	return p;
	
}

Processed register_r(vector<Processed> clouds, boost::shared_ptr<visualization::CloudViewer> viewer) 
{
	if (clouds.size() == 0)
		throw BadArgumentException("empty cloud list");
	if (clouds.size() == 1)
		return clouds[0];
	vector<Processed> next;
	RegistrationParam param;
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	for (size_t i = 0; i < clouds.size()-1; i+=2) {
		auto merged = register_pair(clouds[i], clouds[i+1], param, &transformation, viewer);
		next.push_back(merged);
	}
	if (clouds.size() % 2 == 1)
		next.push_back(clouds.back());
	return register_r(next, viewer);
}

int main(int argc, char **argv)
{
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
		PCL_INFO("%d -> loaded %s, points: %d\n", i, files[i].c_str(), cloud->width * cloud->height);
		// process 
		threads.push_back(thread([i, cloud, &clouds, &clouds_mutex]() mutable {
			ProcessParam param;
			auto processed = process(i, cloud, param);
			lock_guard<mutex> lock(clouds_mutex, adopt_lock);
			clouds.push_back(processed);
		}));
	}
	closedir(dir);
	for (auto& th : threads) 
    	th.join();
	sort(clouds.begin(), clouds.end(), 
		[](const Processed & a, const Processed & b) -> bool {return a.no < b.no;});

	boost::shared_ptr<visualization::CloudViewer> viewer(new visualization::CloudViewer("3D Viewer"));
	auto final = register_r(clouds, viewer);
	viewer->showCloud(final.cloud);

	while (1)
		sleep(1);
	return (0);
}

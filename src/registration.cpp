#include "registration.h"
#include "dbscan.h"

#define PARAMS_FILE "../hyperparams.json"
#define WUI                                    \
	{                                          \
		PCL_INFO(">>> Input to continue <<<"); \
		string s;                              \
		cin >> s;                              \
	}

RegistrationParam load_registration_param(const char *path)
{
	ifstream file;
	file.open(path);
	json j = json::parse(file);
	RegistrationParam p;
	auto block = j["registration"];
	p.init_samples = block["init_samples"];
	p.dbscan_min = block["dbscan"]["min"];
	p.dbscan_eps = block["dbscan"]["eps"];
	block = j["registration"]["init_align"];
	p.ia_min_sample_distance = block["min_sample_distance"];
	p.ia_max_correspondence_distance = block["max_correspondence_distance"];
	p.ia_euclidean_fitness_eps = block["euclidean_fitness_eps"];
	p.ia_outlier_rejection_threshold = block["outlier_rejection_threshold"];
	p.ia_transformation_eps = block["transformation_eps"];
	p.ia_iterations = block["iterations"];
	block = j["registration"]["icp"];
	p.icp_transformation_eps = block["transformation_eps"];
	p.icp_outlier_rejection_threshold = block["outlier_rejection_threshold"];
	p.icp_max_correspondence_distance = block["max_correspondence_distance"];
	p.icp_max_iterations = block["max_iterations"];
	p.icp_euclidean_fitness_eps = block["euclidean_fitness_eps"];
	return p;
}

void printResults(vector<Point> &points, int num_points)
{
	int i = 0;
	printf("Number of points: %u\n"
		   " x     y     z     cluster_id\n"
		   "-----------------------------\n",
		   num_points);
	while (i < num_points)
	{
		for(int j=0;j<16;j++)
			printf("%5.2lf  ", points[i].x[j]);
		printf(": %d\n",points[i].clusterID);
		++i;
	}
}

Processed register_pair(Processed src, Processed dst, ProcessParam process_param, RegistrationParam reg_param, Eigen::Matrix4f *trans, boost::shared_ptr<visualization::CloudViewer> viewer)
{
	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();
	PointCloud<PointXYZRGB>::Ptr aligned_source(new PointCloud<PointXYZRGB>);
	vector<Point> all_alignements;
	float mcd = reg_param.ia_max_correspondence_distance;
	for (size_t i = 0; i < reg_param.init_samples; i++)
	{
		// initial alignment
		SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
		sac_ia.setMinSampleDistance(reg_param.ia_min_sample_distance);
		sac_ia.setMaxCorrespondenceDistance(mcd);
		sac_ia.setEuclideanFitnessEpsilon(reg_param.ia_euclidean_fitness_eps);
		sac_ia.setRANSACOutlierRejectionThreshold(reg_param.ia_outlier_rejection_threshold);
		sac_ia.setTransformationEpsilon(reg_param.ia_transformation_eps);
		sac_ia.setMaximumIterations(reg_param.ia_iterations);
		sac_ia.setInputSource(src.keypoints);
		sac_ia.setSourceFeatures(src.descriptor);
		sac_ia.setInputTarget(dst.keypoints);
		sac_ia.setTargetFeatures(dst.descriptor);
		sac_ia.align(*aligned_source);
		initial_alignment = sac_ia.getFinalTransformation();
		PCL_INFO("[%d - %d] -> computed initial alignment score: %f\n", src.no, dst.no, sac_ia.getFitnessScore());
		mcd -= reg_param.ia_max_correspondence_distance/reg_param.init_samples/2;

		vector<float> vec(initial_alignment.data(),
			initial_alignment.data() + initial_alignment.rows() * initial_alignment.cols());
		Point p;
		p.clusterID = UNCLASSIFIED;
		copy(vec.begin(), vec.end(), p.x);
		all_alignements.push_back(p);
	}
	PCL_INFO("[%d - %d] -> computed %d init alignements\n", src.no, dst.no, reg_param.init_samples);

	DBSCAN ds(reg_param.dbscan_min, reg_param.dbscan_eps, all_alignements);
	ds.run();
	printResults(ds.m_points, ds.getTotalPointSize());
	auto centroid = main_centroid(ds.m_points);
	ostringstream vts; 
	copy(centroid.begin(), centroid.end(), 
        std::ostream_iterator<float>(vts, ", ")); 
	PCL_INFO("[%d - %d] -> computed main cluster centroid: %s\n", src.no, dst.no, vts.str().c_str());
	// vector to matrix4f
	int k = 0;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			initial_alignment(j,i) = centroid[k++];	
	
	transformPointCloud(*src.keypoints, *aligned_source, initial_alignment);
	PointCloud<PointXYZRGB>::Ptr init_aligned_source(new PointCloud<PointXYZRGB>);
	copyPointCloud(*aligned_source, *init_aligned_source);
	*init_aligned_source += *dst.keypoints;
	int32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));
	for (auto &p : init_aligned_source->points)
		p.rgb = rgb;
	viewer->showCloud(init_aligned_source);
	//WUI

	// ICP alignment
	Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity();
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setMaxCorrespondenceDistance(reg_param.icp_max_correspondence_distance);
	icp.setRANSACOutlierRejectionThreshold(reg_param.icp_outlier_rejection_threshold);
	icp.setTransformationEpsilon(reg_param.icp_transformation_eps);
	icp.setMaximumIterations(reg_param.icp_max_iterations);
	icp.setEuclideanFitnessEpsilon(reg_param.icp_euclidean_fitness_eps);
	icp.setInputSource(aligned_source);
	icp.setInputTarget(dst.cloud);
	PointCloud<PointXYZRGB> registration_output;
	icp.align(registration_output);
	final_alignment = icp.getFinalTransformation() * initial_alignment;
	stringstream ss;
	ss << final_alignment;
	auto a = final_alignment.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
	float yaw = a(0, 0);
	float pitch = a(1, 0);
	float roll = a(2, 0);

	Cloud c(new PointCloud<PointXYZRGB>);
	transformPointCloud(*src.cloud, *c, final_alignment);
	PCL_INFO("[%d - %d] -> refined alignment, covergence: %d, score: %f, matrix: \n%s\n",
			 src.no, dst.no, icp.hasConverged(), icp.getFitnessScore(), ss.str().c_str());
	PCL_INFO("roll: %f    pitch: %f    yaw: %f\n", roll, pitch, yaw);
	(*c) += (*dst.cloud);

	*trans = final_alignment;
	viewer->showCloud(c);
	//WUI
	Processed p = post_process(dst.no, c, process_param);
	return p;
}

Processed register_r(vector<Processed> clouds, ProcessParam proc_param, RegistrationParam reg_param, boost::shared_ptr<visualization::CloudViewer> viewer)
{
	if (clouds.size() == 0)
		throw BadArgumentException("empty cloud list");

	if (clouds.size() == 1)
		return clouds[0];

	vector<Processed> next;
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	for (size_t i = 0; i < clouds.size() - 1; i += 2)
	{
		auto merged = register_pair(clouds[i], clouds[i + 1], proc_param, reg_param, &transformation, viewer);
		next.push_back(merged);
	}

	if (clouds.size() % 2 == 1)
		next.push_back(clouds.back());

	return register_r(next, proc_param, reg_param, viewer);
}
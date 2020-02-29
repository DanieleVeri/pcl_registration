#include "registration.h"

#define PARAMS_FILE "../hyperparams.json"
#define WUI {PCL_INFO(">>> Input to continue <<<"); string s; cin >>s;}

RegistrationParam load_registration_param(const char *path) {
	ifstream file;
	file.open(path);
	json j = json::parse(file);
	RegistrationParam p;
	auto block = j["registration"]["init_align"];
	p.ia_min_sample_distance = block["min_sample_distance"];
	p.ia_max_correspondence_distance = block["max_correspondence_distance"];
	p.ia_iterations = block["iterations"];
	block = j["registration"]["icp"];
	p.icp_transformation_eps = block["transformation_eps"];
	p.icp_outlier_rejection_threshold = block["outlier_rejection_threshold"];
	p.icp_max_correspondence_distance = block["max_correspondence_distance"];
	p.icp_max_iterations = block["max_iterations"];
	p.icp_euclidean_fitness_eps = block["euclidean_fitness_eps"];
	return p;
}

Processed register_pair(Processed src, Processed dst, RegistrationParam param, Eigen::Matrix4f* trans, boost::shared_ptr<visualization::CloudViewer> viewer) {
	int iterations = 0;
	while(1) {
		param = load_registration_param(PARAMS_FILE);
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
	
		// ICP alignment
		Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity();
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		icp.setMaxCorrespondenceDistance(param.icp_max_correspondence_distance);
		icp.setRANSACOutlierRejectionThreshold(param.icp_outlier_rejection_threshold);
		icp.setTransformationEpsilon(param.icp_transformation_eps);
		icp.setMaximumIterations(param.icp_max_iterations);
		icp.setEuclideanFitnessEpsilon(param.icp_euclidean_fitness_eps);
		icp.setInputSource(aligned_source);
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
		
		viewer->showCloud(c);
		int stappo;
		cout<<"stappo?";
		cin >>stappo;
		if(!stappo) {
			iterations++;
		} else {
			*trans = final_alignment;
			break;
		}
	}

	Processed p;// = update(dst.no, c, DEFAULT);
	return p;
}

Processed register_r(vector<Processed> clouds, RegistrationParam param, boost::shared_ptr<visualization::CloudViewer> viewer) 
{
	if (clouds.size() == 0)
		throw BadArgumentException("empty cloud list");

	if (clouds.size() == 1)
		return clouds[0];

	vector<Processed> next;
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	for (size_t i = 0; i < clouds.size()-1; i+=2) {
		auto merged = register_pair(clouds[i], clouds[i+1], param, &transformation, viewer);
		next.push_back(merged);
	}

	if (clouds.size() % 2 == 1)
		next.push_back(clouds.back());

	return register_r(next, param, viewer);
}
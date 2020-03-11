#include "registration.h"
#include "dbscan.h"
#include <pcl/registration/ia_kfpcs.h>

Eigen::Matrix4f register_pair(Processed src, Processed dst, ProcessParam process_param, RegistrationParam reg_param, boost::shared_ptr<visualization::CloudViewer> viewer)
{
	Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity();
	PointCloud<PointXYZRGB>::Ptr aligned_source(new PointCloud<PointXYZRGB>);
	vector<Point> all_alignements;
	float mcd = reg_param.ia_max_correspondence_distance;
	PCL_INFO("[%d - %d] -> computing %d init alignements\n", src.no, dst.no, reg_param.init_samples);
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
		PCL_INFO("[%d - %f] ", i, sac_ia.getFitnessScore());
		fflush(stdout);

		mcd -= reg_param.ia_max_correspondence_distance / reg_param.init_samples / 2;

		Point p;
		p.clusterID = UNCLASSIFIED;
		auto v = matrix2vec(initial_alignment);
		copy(v.begin(), v.end(), p.x);
		all_alignements.push_back(p);
	}

	DBSCAN ds(reg_param.dbscan_min, reg_param.dbscan_eps, all_alignements);
	ds.run();
	auto centroid = main_centroid(ds.m_points);
	if (centroid.size() == 0)
		throw BadArgumentException("main alignement cluster not found!!");

	PCL_INFO("[%d - %d] -> computed main alignement cluster centroid: \n", src.no, dst.no);
	initial_alignment = vec2matrix(centroid);
	print_matrix(initial_alignment);
	transformPointCloud(*src.keypoints, *aligned_source, initial_alignment);

	// ICP - refined alignment
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
	PCL_INFO("[%d - %d] -> refined alignment, covergence: %d, score: %f, matrix:",
			 src.no, dst.no, icp.hasConverged(), icp.getFitnessScore());
	print_matrix(final_alignment);

	return final_alignment;
}

Cloud register_r(vector<Processed> clouds, ProcessParam proc_param, RegistrationParam reg_param, boost::shared_ptr<visualization::CloudViewer> viewer)
{
	if (clouds.size() == 0)
		throw BadArgumentException("empty cloud list");

	Eigen::Matrix4f global_clock_trans = Eigen::Matrix4f::Identity(),
					global_counterclock_trans = Eigen::Matrix4f::Identity();
	Cloud merged = extract_main_cluster(clouds[0].cloud, proc_param);
	for (size_t i = 1; i < clouds.size() / 2; i++)
	{
		Cloud aligned_c(new PointCloud<PointXYZRGB>);
		auto clock_trans = register_pair(clouds[i], clouds[i - 1], proc_param, reg_param, viewer);
		global_clock_trans *= clock_trans;
		auto main = extract_main_cluster(clouds[i].cloud, proc_param);
		transformPointCloud(*main, *aligned_c, global_clock_trans);
		(*merged) += (*aligned_c);

		viewer->showCloud(merged);

		Cloud aligned_cc(new PointCloud<PointXYZRGB>);
		auto index = (clouds.size() - i + 1) % clouds.size();
		auto counterclock_trans = register_pair(clouds[clouds.size() - i], clouds[index], proc_param, reg_param, viewer);
		global_counterclock_trans *= counterclock_trans;
		main = extract_main_cluster(clouds[clouds.size() - i].cloud, proc_param);
		transformPointCloud(*main, *aligned_cc, global_counterclock_trans);
		(*merged) += (*aligned_cc);

		viewer->showCloud(merged);
	}

	return merged;
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

Eigen::Matrix4f vec2matrix(vector<float> v)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
	int k = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			matrix(j, i) = v[k++];
	return matrix;
}

vector<float> matrix2vec(Eigen::Matrix4f m)
{
	return vector<float>(m.data(), m.data() + m.rows() * m.cols());
}

void print_matrix(Eigen::Matrix4f m)
{
	stringstream ss;
	ss << m;
	PCL_INFO("\n%s\n", ss.str().c_str());
}
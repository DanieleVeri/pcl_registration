#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
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
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;
using namespace pcl;

int main(int argc, char **argv)
{
	boost::shared_ptr<visualization::CloudViewer> viewer(new visualization::CloudViewer("3D Viewer"));
	PointCloud<PointXYZRGB>::Ptr final_cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr final_keypoints(new PointCloud<PointXYZRGB>);
	PointCloud<pcl::FPFHSignature33>::Ptr final_descriptors(new PointCloud<pcl::FPFHSignature33>);

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

	for(std::size_t i=0; i<files.size(); ++i) {
		auto filename = files[i].c_str();
		if (strcmp(filename, ".") == 0 || strcmp(filename, "..") == 0)
			continue;

		// loading
		cout <<endl<<endl << filename << endl;
		string path = dir_path+filename;
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		io::loadPCDFile(path, *cloud);
		cout << "loaded " << cloud->width * cloud->height << " data points" << std::endl;

		// downsample
		pcl::VoxelGrid<pcl::PointXYZRGB> vox;
		PointCloud<PointXYZRGB>::Ptr cloud_downsampled (new PointCloud<PointXYZRGB>);
		vox.setInputCloud (cloud);
		vox.setLeafSize (0.005f, 0.005f, 0.005);
		vox.filter (*cloud_downsampled);

		// outlier removal
		PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
		StatisticalOutlierRemoval<PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(100);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_filtered);
		console::print_info ("outlier removed\n");

		// find the dominant plane
		float distance_threshold = 0.02;
		float max_iterations = 100;
		SACSegmentation<PointXYZRGB> seg;
		seg.setOptimizeCoefficients(false);
		seg.setModelType(SACMODEL_PLANE);
		seg.setMethodType(SAC_RANSAC);
		seg.setDistanceThreshold(distance_threshold);
		seg.setMaxIterations(max_iterations);
		seg.setInputCloud(cloud_filtered);
		ModelCoefficients::Ptr coefficients(new ModelCoefficients());
		PointIndices::Ptr inliers(new PointIndices());
		seg.segment(*inliers, *coefficients);

		// Extract the inliers
		PointCloud<PointXYZRGB>::Ptr cloud_nobg(new PointCloud<PointXYZRGB>);
		ExtractIndices<PointXYZRGB> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_nobg);
		console::print_info ("dominant plane removed\n");

		// keypoint extraction
		PointCloud<PointXYZRGB>::Ptr cloud_with_keypoints(new PointCloud<PointXYZRGB>);
		float min_scale = 0.002;
		int nr_octaves = 4;
		int nr_scales_per_octave = 5;
		float min_contrast = 1;
		float radius = 0.01;
		SIFTKeypoint<PointXYZRGB, PointWithScale> sift_detect;
		search::KdTree<PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
		sift_detect.setSearchMethod(kd_tree);
		sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
		sift_detect.setMinimumContrast(min_contrast);
		sift_detect.setSearchSurface(cloud_nobg);
		sift_detect.setInputCloud(cloud_nobg);
		sift_detect.setRadiusSearch(radius);
		PointCloud<PointWithScale> keypoints_temp;
		sift_detect.compute(keypoints_temp);
		copyPointCloud(keypoints_temp, *cloud_with_keypoints);
		console::print_info ("keypoint extracted \n");

		// normal estimation
		float normal_radius_search = 0.05;
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
		NormalEstimation<PointXYZRGB, Normal> normal_estimation;
		normal_estimation.setSearchMethod(kd_tree);
		normal_estimation.setRadiusSearch(normal_radius_search);
		normal_estimation.setInputCloud(cloud_nobg);
		normal_estimation.compute(*normals);
		// sift descriptor
		float descriptor_radius_search = 0.1;
		FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh_estimation;
		fpfh_estimation.setSearchMethod(kd_tree);
		fpfh_estimation.setRadiusSearch(descriptor_radius_search);
		fpfh_estimation.setSearchSurface(cloud_nobg);
		fpfh_estimation.setInputNormals(normals);
		fpfh_estimation.setInputCloud(cloud_with_keypoints);
		PointCloud<FPFHSignature33>::Ptr local_descriptors(new PointCloud<FPFHSignature33>);
		fpfh_estimation.compute(*local_descriptors);
		console::print_info ("sift descriptor computed \n");

		if(final_cloud->size() == 0) { // first cloud
			final_cloud = cloud_nobg;
			final_descriptors = local_descriptors;
			final_keypoints = cloud_with_keypoints;
			continue;
		}

		// registration - initial alignment 
		Eigen::Matrix4f initial_alignment = Eigen::Matrix4f::Identity ();
		float min_sample_distance = 0.025;
		float max_correspondence_distance = 0.01;
		int nr_iterations = 500;
		SampleConsensusInitialAlignment<PointXYZRGB, PointXYZRGB, FPFHSignature33> sac_ia;
		sac_ia.setMinSampleDistance (min_sample_distance);
		sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
		sac_ia.setMaximumIterations (nr_iterations);
		sac_ia.setInputCloud (final_keypoints);
		sac_ia.setSourceFeatures (final_descriptors);
		sac_ia.setInputTarget (cloud_with_keypoints);
		sac_ia.setTargetFeatures (local_descriptors);
		PointCloud<PointXYZRGB>::Ptr aligned_source (new PointCloud<PointXYZRGB>);
		sac_ia.align (*aligned_source);
		initial_alignment = sac_ia.getFinalTransformation();
		console::print_info ("computed initial alignment\n");

		// ICP alignment
		Eigen::Matrix4f final_alignment = Eigen::Matrix4f::Identity ();
		float max_correspondence_distance_refine = 0.05;
		float outlier_rejection_threshold = 0.05;
		float transformation_epsilon = 1e-6;
		max_iterations = 1000;
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		icp.setMaxCorrespondenceDistance (max_correspondence_distance);
		icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
		icp.setTransformationEpsilon (transformation_epsilon);
		icp.setMaximumIterations (max_iterations);
		//PointCloud<PointXYZRGB>::Ptr source_points_transformed (new PointCloud<PointXYZRGB>);
		//transformPointCloud (*source, *source_points_transformed, initial_alignment);
		icp.setInputCloud (aligned_source);
		icp.setInputTarget (cloud_nobg);
		PointCloud<PointXYZRGB> registration_output;
		icp.align (registration_output);
		final_alignment = icp.getFinalTransformation () * initial_alignment;
		console::print_info ("refined alignment\n");
		// Transform the source point to align them with the target points
		transformPointCloud (*final_cloud, *final_cloud, final_alignment);

		// final cloud update
		(*final_cloud) += (*cloud_nobg);
		//(*final_descriptors) += (*local_descriptors);
		//(*final_keypoints) += (*cloud_with_keypoints);

		viewer->showCloud(final_cloud);
		
		// keypoint extraction
		min_scale = 0.002;
		nr_octaves = 4;
		nr_scales_per_octave = 5;
		min_contrast = 1;
		radius = 0.01;
		sift_detect.setSearchMethod(kd_tree);
		sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
		sift_detect.setMinimumContrast(min_contrast);
		sift_detect.setSearchSurface(final_cloud);
		sift_detect.setInputCloud(final_cloud);
		sift_detect.setRadiusSearch(radius);
		sift_detect.compute(keypoints_temp);
		copyPointCloud(keypoints_temp, *final_keypoints);
		
		// sift descriptor
		PointCloud<Normal>::Ptr final_normals(new PointCloud<Normal>);
		normal_estimation.setSearchMethod(kd_tree);
		normal_estimation.setRadiusSearch(normal_radius_search);
		normal_estimation.setInputCloud(final_cloud);
		normal_estimation.compute(*final_normals);
		fpfh_estimation.setSearchMethod(kd_tree);
		fpfh_estimation.setRadiusSearch(descriptor_radius_search);
		fpfh_estimation.setSearchSurface(final_cloud);
		fpfh_estimation.setInputNormals(final_normals);
		fpfh_estimation.setInputCloud(final_keypoints);
		fpfh_estimation.compute(*final_descriptors);
		
	}
	closedir(dir);

	viewer->showCloud(final_cloud);

	while (1)
		sleep(1);
	return (0);
}

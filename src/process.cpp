#include "process.h"

Processed process(int no, Cloud cloud, ProcessParam param) {
    // depth filtering
    PointCloud<PointXYZRGB>::Ptr cloud_z_filtered(new PointCloud<PointXYZRGB>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, param.max_z_depth);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_z_filtered);
    PCL_INFO("%d -> z filtered, points: %d \n", no, cloud_z_filtered->points.size());

    // downsample
    VoxelGrid<PointXYZRGB> vox;
    PointCloud<PointXYZRGB>::Ptr cloud_downsampled(new PointCloud<PointXYZRGB>);
    vox.setInputCloud(cloud_z_filtered);
    vox.setLeafSize(param.downsample_grid, param.downsample_grid, param.downsample_grid);
    vox.filter(*cloud_downsampled);
    PCL_INFO("%d -> cloud downsampled, points: %d \n", no, cloud_downsampled->points.size());

    // outlier removal
    PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
    StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setInputCloud(cloud_downsampled);
    sor.setMeanK(param.outlier_meank);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    PCL_INFO("%d -> outlier removed, points: %d \n", no, cloud_filtered->points.size());

    // find the dominant plane
    float max_iterations = 100;
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients(false);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(param.dominant_plane_thr);
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
    PCL_INFO("%d -> dominant plane removed, points: %d \n", no, cloud_nobg->points.size());

    // clustering
    search::KdTree<PointXYZRGB>::Ptr kd_tree_cluster(new search::KdTree<PointXYZRGB>);
    kd_tree_cluster->setInputCloud(cloud_nobg);
    vector<PointIndices> cluster_indices;
    auto min_bound = cloud_nobg->points.size() / 100 * param.clustering_min_perc;
    auto max_bound = cloud_nobg->points.size();
    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setClusterTolerance(param.clustering_tollerance); // meters
    ec.setMinClusterSize(min_bound);
    ec.setMaxClusterSize(max_bound);
    ec.setSearchMethod(kd_tree_cluster);
    ec.setInputCloud(cloud_nobg);
    ec.extract(cluster_indices);
    PointCloud<PointXYZRGB>::Ptr main_cloud_cluster(new PointCloud<PointXYZRGB>);
    for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cluster->points.push_back(cloud_nobg->points[*pit]);
        cluster->width = main_cloud_cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        if (main_cloud_cluster->points.size() < cluster->points.size())
            main_cloud_cluster = cluster;
    }
    PCL_INFO("%d -> clustering completed, points: %d \n", no, main_cloud_cluster->points.size());

    return update(no, main_cloud_cluster, param);
}

Processed update(int no, Cloud cloud, ProcessParam param) {
    // keypoint extraction
    PointCloud<PointXYZRGB>::Ptr cloud_with_keypoints(new PointCloud<PointXYZRGB>);
    SIFTKeypoint<PointXYZRGB, PointWithScale> sift_detect;
    search::KdTree<PointXYZRGB>::Ptr kd_tree(new search::KdTree<PointXYZRGB>());
    sift_detect.setSearchMethod(kd_tree);
    sift_detect.setScales(param.keypoint_min_scale, param.keypoint_nr_octaves, param.keypoint_nr_scales_per_octave);
    sift_detect.setMinimumContrast(param.keypoint_min_contrast);
    sift_detect.setSearchSurface(cloud);
    sift_detect.setInputCloud(cloud);
    sift_detect.setRadiusSearch(param.keypoint_radius);
    PointCloud<PointWithScale> keypoints_temp;
    sift_detect.compute(keypoints_temp);
    copyPointCloud(keypoints_temp, *cloud_with_keypoints);
    PCL_INFO("%d -> keypoint extracted: %d \n", no, cloud_with_keypoints->points.size());

    // normal estimation
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    NormalEstimation<PointXYZRGB, Normal> normal_estimation;
    normal_estimation.setSearchMethod(kd_tree);
    normal_estimation.setRadiusSearch(param.normal_radius);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.compute(*normals);
    // sift descriptor
    FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setSearchMethod(kd_tree);
    fpfh_estimation.setRadiusSearch(param.descriptor_radius);
    fpfh_estimation.setSearchSurface(cloud);
    fpfh_estimation.setInputNormals(normals);
    fpfh_estimation.setInputCloud(cloud_with_keypoints);
    PointCloud<FPFHSignature33>::Ptr main_cluster_descriptors(new PointCloud<FPFHSignature33>);
    fpfh_estimation.compute(*main_cluster_descriptors);
    PCL_INFO("%d -> sift descriptor computed \n", no);

    Processed p;
    p.no = no;
    p.cloud = cloud;
    p.keypoints = cloud_with_keypoints;
    p.descriptor = main_cluster_descriptors;
    return p;
}

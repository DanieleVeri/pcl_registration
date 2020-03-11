#include "mesh.h"

PointCloud<PointNormal>::Ptr smooth_cloud(PointCloud<PointXYZRGB>::Ptr cloud, MeshParam param)
{
    search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
    PointCloud<PointNormal> mls_points;
    MovingLeastSquares<PointXYZRGB, PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(param.mls_order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(param.mls_radius);
    mls.process(mls_points);
    PCL_INFO("final cloud smoothed: %d \n", mls_points.points.size());
    return mls_points.makeShared();
}

int save_vtk(PointCloud<PointNormal>::Ptr smoothed, const char *path, MeshParam param)
{   
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>); 
	cloud->resize(smoothed->size());
	for (size_t i = 0; i < smoothed->points.size(); ++i) 
	{ 
		cloud->points[i].x=smoothed->points[i].x;
		cloud->points[i].y=smoothed->points[i].y;
		cloud->points[i].z=smoothed->points[i].z;
	}
    
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(40);
    n.compute(*normals);
    PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
    concatenateFields(*cloud, *normals, *cloud_with_normals);
    PCL_INFO("normals computed\n");

    GreedyProjectionTriangulation<PointNormal> gp3;
    PolygonMesh triangles;
    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    gp3.setSearchRadius(param.gp3_radius);
    gp3.setMu(param.gp3_mu);
    gp3.setMaximumNearestNeighbors(param.gp3_max_nn);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    return io::saveVTKFile(path, triangles);
}

MeshParam load_mesh_param(const char *path)
{
    ifstream file;
    file.open(path);
    json j = json::parse(file);
    MeshParam p;
    auto block = j["mesh"];
    p.mls_radius = block["mls"]["radius"];
    p.mls_order = block["mls"]["order"];
    p.normal_radius = block["normal_radius"];
    p.gp3_radius = block["gp3"]["radius"];
    p.gp3_mu = block["gp3"]["mu"];
    p.gp3_max_nn = block["gp3"]["max_nn"];
    return p;
}
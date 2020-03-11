#include "process.h"
#include "registration.h"
#include "mesh.h"

#define PARAMS_FILE "../hyperparams.json"
#define PCD_FILE "OUTPUT.pcd"
#define VTK_FILE "OUTPUT.vtk"

int main(int argc, char **argv)
{
	// read data dir
	DIR *dir;
	struct dirent *ent;
	string dir_path = argv[1];
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
    ProcessParam process_param = load_process_param(PARAMS_FILE);
	for (size_t i = 0; i < files.size(); ++i)
	{
		PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		// loading 
		string path = dir_path + files[i];
		io::loadPCDFile(path, *cloud);
		PCL_INFO("%d -> loaded %s, points: %d\n", i, files[i].c_str(), cloud->width * cloud->height);
		// process 
		threads.push_back(thread([i, cloud, process_param, &clouds, &clouds_mutex]() mutable {
			auto processed = process(i, cloud, process_param);
			lock_guard<mutex> lock(clouds_mutex, adopt_lock);
			clouds.push_back(processed);
		}));
	}
	closedir(dir);
	for (auto& th : threads) 
    	th.join();
	sort(clouds.begin(), clouds.end(), 
		[](const Processed & a, const Processed & b) -> bool {return a.no < b.no;});

    // registration
	boost::shared_ptr<visualization::CloudViewer> viewer(new visualization::CloudViewer("3D Viewer"));
    RegistrationParam reg_param = load_registration_param(PARAMS_FILE);
	auto final = register_r(clouds, process_param, reg_param, viewer);
	viewer->showCloud(final);

	// smooth cloud
	MeshParam mesh_param = load_mesh_param(PARAMS_FILE);
	auto smoothed = smooth_cloud(final, mesh_param);
	io::savePCDFileASCII (PCD_FILE, *smoothed);

	// mesh generation
	save_vtk(smoothed, VTK_FILE, mesh_param);
	PCL_INFO("mesh generated: %s\n", VTK_FILE);

	while (!viewer->wasStopped())
        sleep(1);
        
	return EXIT_SUCCESS;
}
 
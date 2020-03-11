#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

using namespace std;
using namespace pcl;

boost::shared_ptr<visualization::CloudViewer> viewer;
Grabber *openniGrabber;
unsigned int filesSaved = 0;
char * file_folder;
bool saveCloud = false;

void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr &cloud)
{
	if (viewer->wasStopped())
		return;

	if (saveCloud)
	{
		stringstream stream;
		stream <<file_folder << (filesSaved < 10 ? "inputCloud0" : "inputCloud") << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else
			PCL_ERROR("Problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}
	viewer->showCloud(cloud);
}

void keyboardEventOccurred(const visualization::KeyboardEvent &event, void *nothing) {
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}

boost::shared_ptr<visualization::CloudViewer> createViewer() {
	boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);
	return (v);
}

int main(int argc, char **argv)
{
	file_folder = argv[1];
	openniGrabber = new OpenNIGrabber();
	if (openniGrabber == 0)
		return -1;

	boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> f = 
		boost::bind(&grabberCallback, _1);
	openniGrabber->registerCallback(f);

	viewer = createViewer();
	openniGrabber->start();

	while (!viewer->wasStopped())
		sleep(1);

	return EXIT_SUCCESS;
}
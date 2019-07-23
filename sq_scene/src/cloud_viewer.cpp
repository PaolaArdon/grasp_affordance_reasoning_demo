#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

#include <pcl/visualization/cloud_viewer.h>

int user_data;

void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main (int argc, char **argv) {
    // read point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1], *cloud);

    // compute centroid
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    Eigen::Matrix<double, 4, 4> transformation_centroid(Eigen::Matrix<double, 4, 4>::Identity ());
    transformation_centroid(0, 3) = - centroid (0);
    transformation_centroid(1, 3) = - centroid (1);
    transformation_centroid(2, 3) = - centroid (2);

    // center data: apply transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_centered, transformation_centroid);

    // blocks until the cloud is actually rendered
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_centered);

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);

    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped ()) {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}

Point Cloud Transformation
===============

Package to transform the coordinate frame of the point cloud from the camera link to the base of PR2 robot. This code is adapted from the [pt_cloud_lab](http://correll.cs.colorado.edu/?p=2807)

## TO RUN TRASNFORM IMAGES
* Basic tutorial on point cloud transformation can be found [here] (http://pointclouds.org/documentation/tutorials/matrix_transform.php)
* Build the package, i.e. `catkin_make` at the root of your workspace
* Export your PR2 `ROS_MASTER_URI`
* `rosrun point_cloud_transformation point_cloud_transformation`
* A `.pcd` image will be saved outside the src folder transformed into world coordinates referenced to the base link of your PR2
* The spatial_filter file helps on getting rid of the background environment




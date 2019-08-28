Grasping regions labels to point clouds
===============

Matlab code to label and save grasping regions from depth images to point clouds. The images come from the Washington D-RGB [dataset](https://rgbd-dataset.cs.washington.edu/) and the Robot Learning Lab [dataset](http://pr.cs.cornell.edu/grasping/).

## If running the code with these datasets
* Download the *grasp_model.mat files from the [website](https://paolaardon.github.io/grasp_affordance_reasoning/) 
* Run `saving_labelled_grasping_regions_pclouds.m` to save the corresponding point cloud image of your d-rgb test.

## If labelling your own dataset
* Organise your data into train, validation and test.
* Run `labelling_grasping_regions.m` to start the labelling of your d-rgb images.

## If using it with the PR2
* The coordinates of the resulting grasping regions are to be input in the utils_grasp.py file to calculate the possible reaching approaches.




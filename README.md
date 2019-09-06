# Grasp Affordance Reasoning

This repository provides the necesary packages to reproduce our work on 'Learning Grasp Affordance Reasoning through Semantic Relations'. If the dataset or the code is used please cite:
```
@inproceedings{ardon2019learning,
  title={Learning Grasp Affordance Reasoning through Semantic Relations},
  author={Paola Ard{\'o}n and  {\`E}ric Pairet, and Ronald P. A. Petrick and Subramanian Ramamoorthy and Katrin S. Lohan},
  journal ={Robotics and Automation Letters (RA-L). To be presented at the International Conference on Intelligent Robots and Systems (IROS)},
  year = {2019},
 publisher={IEEE}
}
```
### Pre-requisites
* [OpenCV](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html)

* [tools_openni2](https://github.com/cvlabbonn/tools_openni2) to save the the RGB and RGB-D images

* [pracmln](http://pracmln.org/index.html) MLN tools in python

* [PCL1.8](https://gitlab.com/EAVISE/publicwiki/wikis/pcl-1.8-installation-ubuntu)

* Download our [dataset](https://paolaardon.github.io/grasp_affordance_reasoning/)

* If wanting to reproduce the work with the PR2, the following [repository](https://github.com/PaolaArdon/pr2_kinetic_simulator) provides a guide on how to install the basics for reaching and grasping with the PR2 in ROS kinetic using different indoor scenes

### The Modules
* mln contains the instructions to learn from the  using Markov Logic Networks

* pt_cloud_transformation contains the instructions to transform the obtained 3D data to the base of the PR2 link

* sq_scene segments out the objects in the scene to be able to extract the grasping regions

* semantic_affordance_grasp_labels allows you to label and learn with your own data, and test images with our model

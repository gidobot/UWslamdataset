# Underwater SLAM dataset

This is a meta repository for UWslam dataset, which can be downloaded from this link:  
[**[UWslam_dataset]**](https://deepblue.lib.umich.edu/data/concern/data_sets/rj430494s?locale=en#read_me_display)

Dataset Creators: G. Billings, M. Johnson-Roberson

Dataset Contact: Gideon Billings gidobot@umich.edu

Funding: NNX16AL08G (NASA), IIS-1830660 (NSF), IIS-1830500 (NSF)

## Key Points

- This is a dataset for developing and testing visual based Simultaenous Localization and Mapping (SLAM) methods in natural underwater environments.
- Dataset includes spiral survey of a shallow reef captured with a diver operated stereo camera.
- Dataset includes 4 synchronized hybrid image sequences from an ROV mounted stereo camera with a manipulator mounted fisheye camera, collected in deep ocean environments.

## Research Overview

Underwater vehicles must be capable of mapping and reconstructing their working environment to perform automated intervention tasks, such as sample collection. This image dataset was collected in natural underwater environments to aid the development and evaluation of underwater SLAM methods. Each image sequence includes globally referenced ground truth camera poses for each image frame. The hybrid image sequences were collected to develop SLAM methods that can fuse images from both vehicle mounted cameras and independently positioned cameras (e.g. manipulator mounted) into the same 3D scene reconstruction. This capability enables active viewpoint acquisition to fill incomplete areas of the scene reconstruction with the independent camera.

## Methodology

Ground truth camera poses for the spiral stereo camera survey were obtained by processing the image sequence through COLMAP. Ground truth camera pose estimates for the hybrid sequences were obtained by distributing AprilTag fiducials on the seafloor before capturing an image sequence and then processing the image sequences through the ROS based [**TagSLAM**](https://berndpfrommer.github.io/tagslam_web/) package.

## Files Contained Here

For each image in a synchronized frame, the naming convention is idx_.png, where idx increments from 0 for each image sequence. For a given image sequence, the timestamp of the idx image frame in seconds is given by the idx row of the time.txt file, indexed from 0 as the first row. The calibration yaml files for the left and right cameras for both the hybrid and stereo spiral sequences are in the format output by the ROS camera_calibration tool using a pinhole projection model (see the following wiki for a description of these parameters: http://wiki.ros.org/camera_calibration_parsers). The fisheye camera was calibrated with the Kalibr toolbox using the equidistant distortion model, and each calibration parameter is explicitely named in the fisheye_calib.yaml file.

For the hybrid sequences, is "left", "right", and "fish", for the left stereo, right stereo, and fisheye images respectively. The camera_poses.txt files give the ground truth camera poses for each camera in a sequence as a globally referenced translation in meters with a rotation quaternion for both the fisheye and stereo cameras (line format is given below).

For the stereo spiral sequence, is "0" and "1" for the left stereo and right stereo images respectively. The gt_poses.txt file gives the ground truth globally referenced translation for each stereo camera frame (note that the camera rotation is not included in the pose file).

The SIFTvoc.txt vocabulary is used with a modified version of the DBoW2 library for indexing and converting images into a bag-of-word representation. Following is a link to a modified version of DBoW2 that supports SIFT features:
[**[DBoW2]**](https://github.com/gidobot/DBoW2)

Below is a sample sequence showing the AprilTag detector and TagSLAM estimated camera poses for a hybrid sequence, vizualizing the fisheye images in greyscale.

![Output sample](https://github.com/gidobot/gifs/raw/master/VisPose_AprilSLAM.gif)

Below is a sample left stereo image from the shallow reef spiral survey dataset and the COLMAP reconstruction of the full survey.

![uwslam_reef](https://github.com/gidobot/UWslamdataset/assets/6425217/c47dabcf-d44a-4a85-af18-0a387de87caf | width=100)

The dataset is organized by the following file structure
```
UWslam_dataset
└───hybrid -> folder for hybrid image sequences
│ └───calibration
│ │ │ fisheye_calib.yaml -> calibration file for fisheye in OpenCV format
│ │ │ left_camera.yaml -> calibration file for left stereo camera in ROS camera_calibration format
│ │ │ right_camera.yaml -> calibration file for right stereo camera in ROS camera_calibration format
│ └───
│ │ camera_poses.txt -> ground truth hybrid camera poses in line format: idx fish_tx fish_ty fish_tz fish_qw fish_qx fish_qy fish_qz stereo_tx stereo_ty stereo_tz stereo_qw stereo_qx stereo_qy stereo_qz
│ │ times.txt -> image timestamps where row indexed from 0 corresponds to frame idx
│ └───images/raw -> folder containing rectified stereo and raw fisheye images
└───stereo -> folder for stereo survey sequence
│ │ gt_poses.txt -> ground truth trajectory in line format: timestamp stereo_tx stereo_ty stereo_tz
│ │ times.txt -> image timestamps
│ └───calibration
│ │ │ left_camera.yaml -> calibration file for left stereo camera
│ │ │ right_camera.yaml -> calibration file for right stereo camera
│ └───images -> folder containing rectified stereo images
└───sift_vocabulary
│ │ SIFTvoc.txt -> SIFT DBOW2 vocabulary trained on underwater images
```
## Use and Access
This data set is made available under a Creative Commons Public Domain license (CC0 1.0).

## To Cite Data
```
@article{billings2022hybrid,
  title={Hybrid Visual SLAM for Underwater Vehicle Manipulator Systems},
  author={Billings, Gideon and Camilli, Richard and Johnson-Roberson, Matthew},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={3},
  pages={6798--6805},
  year={2022},
  publisher={IEEE}
}
```

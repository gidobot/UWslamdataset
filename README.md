# UWslam
This is a meta repository for UWslam dataset, which can be downloaded from this Google Drive link:  
[**[UWslam]**](https://drive.google.com/file/d/1mZYeBiceVeo9dRYaCuJBaY63NufiA_fB/view?usp=sharing)

UWslam is a dataset for underwater stereo and hybrid monocular fisheye + stereo SLAM in natural seafloor environments. The dataset includes a spiral survey of a shallow reef captured with a diver operated stereo rig, and 4 hybrid image sequences captured with a deep ocean ROV in different deep ocean environments. Ground truth pose estimates for the spiral stereo trajectory were obtained by processing the images through COLMAP. Ground truth pose estimates for the hybrid sequences were obtained using AprilTag based SLAM, on tags that were distributed onto the seafloor while the image sequences were captured.

## File Structure
 ```
UWslam_dataset
└───hybrid 
│   └───calibration  
│   │   │   fisheye_calib.yaml -> calibration file for fisheye  
│   │   │   left_camera.yaml -> calibration file for left stereo camera
│   │   │   right_camera.yaml -> calibration file for right stereo camera
│   └───<set name>
│       │   camera_poses.txt -> ground truth camera poses
│       │   times.txt -> image timestamps
│       └───images/raw -> folder containing rectified stereo and raw fisheye images
└───stereo 
│   └───calibration  
│   │   │   left_camera.yaml -> calibration file for left stereo camera
│   │   │   right_camera.yaml -> calibration file for right stereo camera
│   └───images -> folder containing rectified stereo images
│   gt_poses.txt -> ground truth trajectory
│   times.txt -> image timestamps
└───stereo
│   │   SIFTvoc.txt -> SIFT DBOW2 vocabulary trained on underwater images
```
<!--
## Overview

Sample annotated sequence, showing center rectified images for visualization of the model handle projections

![Output sample](https://github.com/gidobot/gifs/raw/master/VisPose_Reviewer.gif)

The dataset was annotated using the VisPose annotation tool, which can also be used to review the annotations:  
[**[VisPose]**](https://github.com/gidobot/VisPose)

The sequence consistent camera poses for input to the VisPose annotation tool were generated using the ROS based [**TagSLAM**](https://berndpfrommer.github.io/tagslam_web/) package. Below is a sample sequence showing the AprilTag detector and TagSLAM estimated camera poses

![Output sample](https://github.com/gidobot/gifs/raw/master/VisPose_AprilSLAM.gif)

## Citation
If you use this dataset, we request you to cite the following work.
```
@ARTICLE{9091344,
 author={G. {Billings} and M. {Johnson-Roberson}},
 journal={IEEE Robotics and Automation Letters},
 title={SilhoNet-Fisheye: Adaptation of A ROI-Based Object Pose Estimation Network to Monocular Fisheye Images},
 year={2020},
}
```
 -->
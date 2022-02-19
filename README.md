# UWslam_dataset
This is a meta repository for UWslam dataset, which can be downloaded from this Google Drive link:  
[**[UWslam_dataset]**](https://drive.google.com/file/d/1jV2dmvPZpjScwCPO0kNTN4jIHaDL8FU-/view?usp=sharing)

UWslam is a dataset for underwater stereo and hybrid monocular fisheye + stereo SLAM in natural seafloor environments. The dataset includes a spiral survey of a shallow reef captured with a diver operated stereo rig and 4 hybrid image sequences captured with a deep ocean ROV in different deep ocean environments. Ground truth pose estimates for the spiral stereo trajectory were obtained by processing the image sequence through COLMAP. Ground truth pose estimates for the hybrid sequences were obtained by distributing fiducials on the seafloor before capturing an image sequence and processing the image sequences with the ROS based [**TagSLAM**](https://berndpfrommer.github.io/tagslam_web/) package.

## File Structure
 ```
UWslam_dataset
└───hybrid -> folder for hybrid image sequences
│   └───calibration  
│   │   │   fisheye_calib.yaml -> calibration file for fisheye  
│   │   │   left_camera.yaml -> calibration file for left stereo camera
│   │   │   right_camera.yaml -> calibration file for right stereo camera
│   └───<set name>
│       │   camera_poses.txt -> ground truth hybrid camera poses in line format: timestamp fish_tx fish_ty fish_tz fish_qw fish_qx fish_qy fish_qz stereo_tx stereo_ty stereo_tz stereo_qw stereo_qx stereo_qy stereo_qz
│       │   times.txt -> image timestamps
│       └───images/raw -> folder containing rectified stereo and raw fisheye images
└───stereo -> folder for stereo survey sequence
│   │   gt_poses.txt -> ground truth trajectory in format: tx ty tz
│   │   times.txt -> image timestamps
│   └───calibration  
│   │   │   left_camera.yaml -> calibration file for left stereo camera
│   │   │   right_camera.yaml -> calibration file for right stereo camera
│   └───images -> folder containing rectified stereo images
└───sift_vocabulary
    │   SIFTvoc.txt -> 1 million word SIFT DBOW2 vocabulary trained on underwater images
```

## Overview

For the hybrid sequences, the ground truth camera poses are given as the transform from the respectful camera frame to the world frame. Hybrid images were collected at 3Hz.

For the stereo dataset, the ground truth poses are given with respect to the COLMAP world frame. The stereo frames were collected at 5Hz.

The utils folder provides scripts for evaluating results on the stereo dataset and the hybrid image dataset.

The SIFT DBoW2 vocabulary can be used with a modified version of the DBoW2 library available here:
[**[DBoW2]**](https://github.com/gidobot/DBoW2)

<!--
Sample annotated sequence, showing center rectified images for visualization of the model handle projections

![Output sample](https://github.com/gidobot/gifs/raw/master/VisPose_Reviewer.gif)

The dataset was annotated using the VisPose annotation tool, which can also be used to review the annotations:  
[**[VisPose]**](https://github.com/gidobot/VisPose)

The sequence consistent camera poses for input to the VisPose annotation tool were generated using the ROS based [**TagSLAM**](https://berndpfrommer.github.io/tagslam_web/) package. Below is a sample sequence showing the AprilTag detector and TagSLAM estimated camera poses

![Output sample](https://github.com/gidobot/gifs/raw/master/VisPose_AprilSLAM.gif)
 -->

## Citation
If you use this dataset, we request you to cite the following work.
```
@article{billings2021hybrid,
  title={Hybrid Visual SLAM for Underwater Vehicle Manipulator Systems},
  author={Billings, Gideon and Camilli, Richard and Johnson-Roberson, Matthew},
  journal={arXiv preprint arXiv:2112.03826},
  year={2021}
}
```

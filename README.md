# MonoVO
Monocular visual odometry code in MatLAB using KITTI dataset

# Intro
This repository is a monocular visual odometry pipeline written in MatLAB and uses MatLAB built in functions to perform pure VO on the [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) dataset. This code draws from Avi Singh's [stereo visual odometry pipeline](https://github.com/avisingh599/vo-howard08/blob/master/README.md). Additionaly, his blog post on [VO](https://avisingh599.github.io/vision/visual-odometry-full/) I found to be a really good resource.

# Start
+ Clone this repository and add everything to your path. The KITTI images that this repository uses can be dowloaded [here](https://umich.box.com/s/6w93aipmshjyjropkrd8ouzk17iamyaj). Put this '00' folder in the 'data' folder. To run this code simply call the script imageCallMono.m. This script will run for a few minutes, as is it is set to operate on 500 image pairs. 
+ Once it finishes there are two key outputs that you should save to the data folder:(1) 'result_pos' and (2)'final_poses'. The former is the incremental translations and the latter is the cumualative translations. 
+ Run the script groundTruthVO.m to plot the x-z trajectory of the vehicle, the overall error, and the incremental error in pos(x,y,z). Running this script as is will work and will plot the recovered trajectory that I saved using SURF features which is stored in the data folder. 

# Code Explained
note: a lot of detail is described in comments in the code.

Starting with imageCallMono.m

+ The camera calibration and ground trueth poses can be found on the KITTI odometry page. The documentation for the camera calibration explains how this information is stored but for the purposes of this code the camera intrinsic matrix is stored as variable K. 
```
%IntrinsicMatrixl = Pl(1:3,1:3);
K1 = Pl(1:3,1:3); (line 20-21)
```
Similarly, I have stored the ground trueth poses in M.mat which can be found in the data folder. 

+ You can also control the amount of images you want the code to operate. 
```
% n is the number of images one wishes to consider
%n = 4540;
n=500; (lines 29-31)
```

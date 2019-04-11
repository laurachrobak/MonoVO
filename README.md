# MonoVO
Monocular visual odometry code in MatLAB using KITTI dataset
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/images/VOplot.png?raw=true"/>
</p>

# Intro

This repository is a monocular visual odometry pipeline written in MatLAB. It uses MatLAB built in functions to perform pure VO on the [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) dataset. This code draws from Avi Singh's [stereo visual odometry pipeline](https://github.com/avisingh599/vo-howard08/blob/master/README.md) and from the MatLAB [Monocular Visual Odometry](https://www.mathworks.com/help/vision/examples/monocular-visual-odometry.html) example code. Additionally, I found his blog post on [VO](https://avisingh599.github.io/vision/visual-odometry-full/) to be a really useful resource.

# Start
+ Clone this repository and add everything to your path. The KITTI images that this repository uses can be downloaded [here](https://umich.box.com/s/6w93aipmshjyjropkrd8ouzk17iamyaj). Put everything from this download in the 'data' folder. To run this code simply call the script imageCallMono.m. This script will run for a few minutes, as is it is set to operate on 500 image pairs. 
+ Once it finishes there are two key outputs that you should save to the data folder:(1) 'result_pos' and (2)'final_poses'. The former is the incremental translations and the latter is the cumulative translations. 
+ Run the script groundTruthVO.m to plot the x-z trajectory of the vehicle, the overall error, and the incremental error in pos(x,y,z). Running this script as is will work and will plot the recovered trajectory that I saved using SURF features which is stored in the data folder. If you want to run it with the trajectory you just created in the last step make sure to load the result_pos and final_poses that you saved at the begining of this script. 

# Code Explained
note: a lot of detail is described in comments in the code.

Starting with ...

# imageCallMono.m

+ The camera calibration and ground truth poses can be found on the KITTI odometry page. The documentation for the camera calibration explains how this information is stored but for the purposes of this code the camera intrinsic matrix is stored as variable K. 
```
%IntrinsicMatrixl = Pl(1:3,1:3);
K1 = Pl(1:3,1:3); (line 20-21)
```
Similarly, I have stored the ground truth poses in M.mat which can be found in the data folder. 

+ You can also control the amount of images you want the code to operate on. 
```
% n is the number of images one wishes to consider
%n = 4540;
n=500; (lines 29-31)
```
+ multipleVOexzapleMono operates on two images- one at times t and the other at time t+1. This function returns an incremental orientation and rotation between the two frames. The position relative to the global frame is then calculated and stored. 

pose in global frame:
```
pos = pos + Rpos * tr;
Rpos = R * Rpos; (lines 66-67)
final_poses(:,i) = pos;(line 72)
```
incremental pose:
```
result_pos(:,i) = tr;
result_Rpos(:,:,i) = R;
```
# multipleVOexamplemono.m
This function is just an intermediary for findFeaturesMono() and EstimateRelativePose().

# findFeaturesMono.m

This function takes in the two images, at times t and t+1, and outputs the features that are then used to estimate the relative pose between the two frames. 

+ The first step is to detect the features within the image. Matlab has different built in function for various feature detectors (i.e SURF, Harris, BRISK). 
```
 points_1 = detectSURFFeatures(I_1, 'MetricThreshold', 1000);
 points_2 = detectSURFFeatures(I_2, 'MetricThreshold', 1000); (lines 6-7)
 ```
+ Once the features are detected a feature discriptor is extracted. Here I have set the parameter 'Upright' to true because rotations of the vehicle frame-to-frame are small but you can turn this argument off if you want. 
```
[f_1,vpts_1] = extractFeatures(I_1,points_1,'Upright', true);
[f_2,vpts_2] = extractFeatures(I_2,points_2,'Upright', true); (lines 20-21)
```
+The feature descriptors are then matched frame-to-frame. Setting the 'Unique' argument to true prevents multiple features in frame t from matching to the same feature in frame t+1.  
```
indexPairs = matchFeatures(f_1,f_2,'Unique', true); (line 30)
```
+ If the plotBoolean is set to true you can also visualize the matches you are getting, which is a good way to see how well the algorithm is performing on your images. 
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/images/Left-t-features.png?raw=true"/>
</p>
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/images/Left-tplus1-features.png?raw=true"/>
</p>
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/images/matched-features.png?raw=true"/>
</p>

# EstimateRelativePose.m
Finally, the function EstimateRelativePose takes in the matched features and outputs the transformation. This function first estimates the essential matrix between the two frames.
```
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        cameraParams,'MaxNumTrials' ,1000,'Confidence',99,'MaxDistance' ,.1); (lines 17-18)
```
The inliers from this estimation are used to calculate the relative camera pose. 
```
    [orientation, location, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints1,...
        inlierPoints2);(lines 37-39)
```
I tried a few different algorithms to recover the relative camera pose but the structure from the matlab monocular visual odometry example code seemed to work best and this function is essentially the 'helperEstimateRelativePose.m' from there example.

# groudTruthVO.m
This function plots the x-z trajectory of the vehicle, the overall error, and the incremental error in pos(x,y,z). One problem we run into when working with monocular images is scale ambiguity. Because this pipeline is monocular the trajectory will not be to the correct scale. Different methods exists to resolve the scale. One common way is to put an object of known size in the first frames and use this known length to solve for the scaling factor. Here I have just scaled the trajectory to match the ground truth trajectory. This is how they do it in the helperNormalizeViewSet.m scripts from MatLABs monocular visual odometry example. 

Additionally, the incremental error is much more indicative of the performance of the visual odometry algorithm than the overall error is. 
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/error_plot_in1.jpg?raw=true"/>
</p>
<p align="center">
  <img src="https://github.com/laurachrobak/MonoVO/blob/master/errorplot_BRISK.jpg?raw=true"/>
</p>

# Improvements
Some things I would like to add:
* make this code work for stereo pairs
  * try different types of pose estimation (i.e. 2d-3d or 3d-3d). This code only uses 2d-2d. 
* add bundle adjustment



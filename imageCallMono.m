% Take in two rectified rgb left camera images from Kitti

%% Initialize
clear all
% Load Images
image0name = dir('./data/00/image_0/*.png');
image1name = dir('./data/00/image_1/*.png');
% Folder 'image_3' holds right images, 'image_2' holds left images
% png 0 corresponds to time t, png 2 corresponds to time tplus1

% Read in calibration from Kitti odometry
calibname = './data/calib.txt';
T = readtable(calibname, 'Delimiter', 'space', 'ReadRowNames', true, 'ReadVariableNames', false);
A = table2array(T);

% Pl, Camera matrix corresponding to the left camera
Pl = vertcat(A(1,1:4), A(1,5:8), A(1,9:12));

% Pull Intrinsic (K) from projection matrix
%IntrinsicMatrixl = Pl(1:3,1:3);
K1 = Pl(1:3,1:3);

% Make cameraparameter class required for future functions
%cameraParamsl = cameraParameters('IntrinsicMatrix',IntrinsicMatrixl);

% Assign plotBoolean to true if plots are desired
plotBoolean = 0;

% n is the number of images one wishes to consider
%n = 4540;
n=500;
% Initialize motion estimation
Rpos = eye(3);
pos = [0;0;0];


for i = 1:n%suppose there are n images
    %Image folder 01 is right
    %Image folder 00 is left
    
    file_name0 = strcat('./data/00/image_0/',image0name(i).name);
    file_name1 = strcat('./data/00/image_1/',image1name(i).name);
    I1_l = imread(file_name0);
    I1_r = imread(file_name1);
    file_name01 = strcat('./data/00/image_0/',image0name(i+1).name);
    file_name11 = strcat('./data/00/image_1/',image1name(i+1).name);
    I2_l = imread(file_name01);
    I2_r = imread(file_name11);
    dims = size(I2_l);
    cam1 = cameraIntrinsics([K1(1, 1), K1(2,2)], [K1(1, 3), K1(2, 3)], dims);
    %Use to display current iteration working images
%     figure;
%     imshow(I1_l)
%     title('previous')
%     figure;
%     imshow(I2_l)
%     title('current')
%     
    [relativeOrientation,relativeLocation] = ...
        multipleVOexampleMono(I1_l, I2_l, cam1, plotBoolean);

    R = relativeOrientation;
    tr = relativeLocation';

    %% Estimated pose relative to global frame at t = 0
    pos = pos + Rpos * tr;
    Rpos = R * Rpos;

    result_pos(:,i) = tr;
    result_Rpos(:,:,i) = R;
    
    final_poses(:,i) = pos;
   

end

    
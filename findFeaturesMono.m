function [ inliers1,inliers2] = ...
    findFeaturesAndMatchMono(I_1, I_2, plotBoolean)
%% Detect SURF features
% points_1 = detectHarrisFeatures(I_1);
% points_2 = detectHarrisFeatures(I_2);
 points_1 = detectSURFFeatures(I_1, 'MetricThreshold', 1000);
 points_2 = detectSURFFeatures(I_2, 'MetricThreshold', 1000);

%% Extract SURF features 

% Select a subset of features, uniformly distributed throughout the image.
% numPoints = 300;
% prevPoints = selectUniform(points_1, numPoints, size(I_1));
% currentPoints = selectUniform(points_2, numPoints, size(I_2));

% [f_1,vpts_1] = extractFeatures(I_1,prevPoints,'Upright', true);
% [f_2,vpts_2] = extractFeatures(I_2,currentPoints,'Upright', true);

%Upright makes the SURF features rotationally variaent 
[f_1,vpts_1] = extractFeatures(I_1,points_1,'Upright', true);
[f_2,vpts_2] = extractFeatures(I_2,points_2,'Upright', true);
% [f_1,vpts_1] = extractFeatures(I_1,points_1);
% [f_2,vpts_2] = extractFeatures(I_2,points_2);
% f_1, f_2 are the feature descriptors, vpts_1/2 are the
% feature objects containing information about SURF feature detection
% vpts_1/2 is smaller than points_1 because some points are lost if they
% are, for instance, too close tot the edge

%% Match features
indexPairs = matchFeatures(f_1,f_2,'Unique', true) ;

inliers1= vpts_1(indexPairs(:,1));
inliers2 = vpts_2(indexPairs(:,2));

%decriptors_1/2 are feature descriptors, 
%use if needed for future matching
% descriptors_1 = f_1(indexPairs(:,1),:);
% descriptors_2 = f_2(indexPairs(:,2),:);

if plotBoolean == 1
figure;
hold on;
J = insertMarker(I_1,inliers1,'o','size',7,'color','red');
imshow(J)
title('Features Left Image at time T');
hold off;

figure;
hold on;
G = insertMarker(I_2,inliers2,'size',7);
imshow(G)
title('Features Left Image at time Tplus1');
hold off;

figure; 
hold on;
showMatchedFeatures(I_1, I_2, inliers1, inliers2);
title('matched features in left images');
legend('t','tplus1')
hold off;
end




end

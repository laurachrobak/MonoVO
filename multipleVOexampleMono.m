function [relativeOrientation,relativeLocation] = multipleVOexampleMono(I_1, I_2, cameraParamsl, plotBoolean)

% Finding features for current frame
[ inliers1,inliers2] = ...
    findFeaturesMono(I_1, I_2, plotBoolean)

[orientation, location, inlierIdx] = ...
    EstimateRelativePose(inliers1, inliers2, cameraParamsl);

relativeOrientation = orientation;
relativeLocation = location;

end

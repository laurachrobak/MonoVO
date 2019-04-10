function [orientation, location, inlierIdx] = ...
    EstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams)

if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end

for i = 1:100
    validPointFraction = 0;
    y = 0;
    
    % Estimate the essential matrix.    
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        cameraParams,'MaxNumTrials' ,1000,'Confidence',99,'MaxDistance' ,.1);

    % Make sure we get enough inliers
    r = sum(inlierIdx) / numel(inlierIdx);
    if r < .3
        continue;
    end
    
    % Get the epipolar inliers.
    y = sum(inlierIdx);
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);    
    
    % Compute the camera pose from the fundamental matrix. Use half of the
    % points to reduce computation.
%     [orientation, location, validPointFraction] = ...
%         relativeCameraPose(E, cameraParams, inlierPoints1(1:2:end, :),...
%         inlierPoints2(1:2:end, :));

    [orientation, location, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints1,...
        inlierPoints2);
    p = validPointFraction;
    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if validPointFraction > .8
       return;
    end
    
end

% After 100 attempts validPointFraction is still too low.
%error('Unable to compute the Essential matrix');
 orientation = eye(3);
 location = [0,0,0]

end


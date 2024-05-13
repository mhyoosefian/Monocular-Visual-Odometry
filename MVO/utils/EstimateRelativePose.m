function [orientation, location, inlierIdx] = ...
    EstimateRelativePose(matched1, matched2, intrinsics, K)

if ~isnumeric(matched1)
    matched1 = matched1.Location;
end

if ~isnumeric(matched2)
    matched2 = matched2.Location;
end

for i = 1:20
    
    [~, inlierIdx] = estimateFundamentalMatrix(matched1, matched2,...
        'Method', 'RANSAC', 'NumTrials', 3000, 'DistanceThreshold', 1e-3);

    % Make sure we get enough inliers
    r = sum(inlierIdx) / numel(inlierIdx);
    if r > 0.8
        continue;
    end
    
    % Get the epipolar inliers.
    inlierPoints1 = matched1(inlierIdx, :);
    inlierPoints2 = matched2(inlierIdx, :);   
    
    E = estimateEssentialMatrix([inlierPoints1.'; ones(1,length(inlierPoints1))],...
        [inlierPoints2.'; ones(1,length(inlierPoints2))], K, K);
    
    [R,u3] = decomposeEssentialMatrix(E);
    [R,T,total_points_in_front_best] = disambiguateRelativePose(R,u3,[inlierPoints1.'; ones(1,length(inlierPoints1))],...
        [inlierPoints2.'; ones(1,length(inlierPoints2))], K, K);

    
    % Nonlinear Refinement
    [R, T] = reprojectionErrorOptimization(R, T, [inlierPoints1.'; ones(1,length(inlierPoints1))],...
                                            [inlierPoints2.'; ones(1,length(inlierPoints2))], K, K);

    orientation = R;
    location = T.';


    p = total_points_in_front_best/length(inlierPoints1)/2;

    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if p > 0.85 && r < 0.8
       return;
    end
    
end

% After 20 attempts validPointFraction is still too low.
%error('Unable to compute the Essential matrix');
 orientation = eye(3);
 location = [0,0,0];


function [R,T] = processFrames(prevImg, currImg, intrinsics, K)
%%
num_keypoints = 1000;
n = 800;
imageSize = size(prevImg);

points_1 = detectHarrisFeatures(prevImg, 'MinQuality', 0.01);
points_1 = points_1.selectStrongest(num_keypoints);
points_1 = selectUniform(points_1,n,imageSize);
points1 = points_1.Location;

pointTracker = vision.PointTracker('MaxBidirectionalError', .1, 'BlockSize', [21 21], 'MaxIterations', 50);
% pointTracker = vision.PointTracker('MaxBidirectionalError',1);
initialize(pointTracker, points1, prevImg);
[points2,point_validity,scores] = pointTracker(currImg);
matched1 = [0 0];
matched2 = [0 0];
num_matches = length(scores);
for i=1:num_matches
    if point_validity(i) == 1
        matched1 = [matched1; points1(i,:)];
        matched2 = [matched2; points2(i,:)];
    end 
end
matched1 = matched1(2:end,:);
matched2 = matched2(2:end,:);

%%
[R, T, inlierIdx] = ...
    EstimateRelativePose(matched1, matched2, intrinsics, K);

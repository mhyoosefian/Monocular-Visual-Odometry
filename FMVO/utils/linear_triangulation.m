function [optimized_point, validity] = linear_triangulation(RCG, pGC, features, max_condition, K)
% This function computes an estimation of a 3D point using its pixel
% coordinates observed in "n" number of consecutive camera frames.

% INPUTS: 1- RCG (3 by 3n): camera rotations wrt global frame

%         2- pGC (3 by n): camera position wrt global frame

%         3- features (1 by 2n): pixel coordinates of the point in camera
%            frames

%         4- min_dist (1 by 1): a tuning parameter. after triangulation,
%            the 3D point is transformed from global frame to all camera
%            frames and checked to be in front of each camera, meaning that
%            it should has positive depth. To make the test more stricter,
%            we expect the point to have a minimum distance from the camera
%            plane (having a positive depth is necessary but not sufficient)

%         5- max_condition (1 by 1): another tuning parameter. When the
%            condition number is too high, the result is not trustworthy,
%            since the point is probably too far to be triangulated correctly 

%         6- K (3 by 3): camera intrinsics

% OUTPUTS: 1- optimized_point (3 by 1): the triangulated point in global
%             frame

%          2- validity (1 by 1): a scalar to determine whether the
%             triangulated point is relaible or not. it is either "1" or "0"

validity = 1;
num_measurements = size(pGC,2);     % this is "n"
assert(num_measurements == size(features,2)/2, 'number of features and measurements mismatch');
A = zeros(2*num_measurements, 3);
b = zeros(2*num_measurements, 1);
for idx = 1:num_measurements
    R = RCG(:,3*idx-2:3*idx);
    t = pGC(:,idx);
    tCG = -R*t;
    temp = K \ [features(1,2*idx-1); features(1,2*idx); 1]; % normalizing features
    u = temp(1);
    v = temp(2);

    A(2*idx-1, :) = u*R(3,:) - R(1,:);
    A(2*idx, :) = v*R(3,:) - R(2,:);

%     A(2*idx-1, :) = -(u*R(3,:) - R(1,:));  %THIS IS WRONG
%     A(2*idx, :) = -(v*R(3,:) - R(2,:));
    
    b(2*idx-1, :) = -(u*tCG(3) - tCG(1));
    b(2*idx, :) = -(v*tCG(3) - tCG(2));

%     b(2*idx-1, :) = -(u*tCG(3) - tCG(1));  %THIS IS WRONG
%     b(2*idx, :) = -(v*tCG(3) - tCG(2));
end
triangulated_point = lsqminnorm(A,b);
S = svd(A);

% check the solution to be good enough
condition_number = S(1)/S(end);
if condition_number > max_condition
    validity = 0;
end

% check the point to be in front of all camera frames
for idx = 1:num_measurements
    R = RCG(:,3*idx-2:3*idx);
    t = pGC(:,idx);
    tCG = -R*t;
    point_in_camera_frame = R*triangulated_point + tCG;
    if point_in_camera_frame(3) < 0
        validity = 0;
    end
end

% converting the triangulated point into inverse-depth parametrization
x = triangulated_point(1);
y = triangulated_point(2);
z = triangulated_point(3);
inverse_depth_point = [x/z; y/z; 1/z];

% solving a least-square minimization problem to find the optimized
% inverse-depth 3d point
alpha = inverse_depth_point(1);
beta = inverse_depth_point(2);
rho = inverse_depth_point(3);
rep_err = @(x) Error(x, RCG, pGC, features, K);
initial_condition = [alpha; beta; rho];
options = optimoptions(@lsqnonlin, 'Display','off','MaxIter', 100);
optimized_point_inverse_depth = lsqnonlin(rep_err, initial_condition, [], [], options);

% converting back the inverse-depth parametrization to normal coordinates
alpha = optimized_point_inverse_depth(1);
beta = optimized_point_inverse_depth(2);
rho = optimized_point_inverse_depth(3);
optimized_point = [alpha/rho; beta/rho; 1/rho];

% check the optimized point to be in front of all camera frames
for idx = 1:num_measurements
    R = RCG(:,3*idx-2:3*idx);
    t = pGC(:,idx);
    tCG = -R*t;
    point_in_camera_frame = R*optimized_point + tCG;
    if point_in_camera_frame(3) < 0
        validity = 0;
    end
end
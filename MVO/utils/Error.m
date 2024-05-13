function reprojection_error = Error(x, RCG, pGC, features, K)
% This function calculates the reprojection error of a triangulated 3D
% point in inverse depth parametrization.
% INPUTS: 1- x (3 by 1): the triangulated 3D point in inverse depth
%         parametrization

%         2- RCG (3 by 3n): rotation matrices of "n" camera frames wrt
%         global frame

%         3- pGC (3 by n): position of "n" camera frames wrt global frame

%         4- features (1 by 2n): pixel coordinates of the triangulated
%         point in "n" frames

%         5- K (3 by 3): camera intrinsics

% OUTPUT: reprojection_error (2n by 1): each 2 rows containt reprojection
%         error of the point in one image

num_measurements = size(pGC,2);
alpha = x(1);                   % for more information on inverse depth parametrization, see MSCKF paper's appendix
beta = x(2);
rho = x(3);
for idx = 1:num_measurements
    R = RCG(:,3*idx-2:3*idx);
    t = pGC(:,idx);
    tCG = -R*t;
    h = R*[alpha; beta; 1] + rho*tCG;
    reprojected_point = 1/h(3)*[h(1) h(2)];
    temp = K \ [features(1,2*idx-1); features(1,2*idx); 1];
    normalized_features = [temp(1) temp(2)];
    reprojection_error(2*idx-1:2*idx,1) = reprojected_point.' - normalized_features.';
end
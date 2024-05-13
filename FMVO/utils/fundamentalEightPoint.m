function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix

[dim, NumPoints] = size(p1);
[dim2, NumPoints2] = size(p2);

% Sanity checks
assert(dim == dim2 && NumPoints == NumPoints2, 'Size mismatch of input points');
assert(dim == 3, 'Input arguments are not 2D points');
assert(NumPoints >= 8, 'Insufficient number of points to compute fundamental matrix (need >=8)');

% Compute the measurement matrix A of the linear homogeneous system whose
% solution is the vector representing the fundamental matrix.
A = zeros(NumPoints,9);
for i=1:NumPoints
    A(i,:) = kron(p1(:,i),p2(:,i)).';
end

% "Solve" the linear homogeneous system of equations A*f = 0.
[~, ~, V] = svd(A,0);
F = reshape(V(:,9),3,3);

% Enforce det(F)=0 by projecting F onto the set of 3x3 singular matrices
[u, s, v] = svd(F);
s(3,3) = 0;
F = u*s*v';


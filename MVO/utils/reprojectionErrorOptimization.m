function [optimal_R, optimal_T] = reprojectionErrorOptimization(R, T, points0_h, points1_h, K1, K2)


M1 = K1 * eye(3,4);
M2 = K2 * [R, T];

% Points in the world frame
% ASSUMPTION: points behind both cameras are the same.
P_C1 = linearTriangulation(points0_h,points1_h,M1,M2);
outlier0_idx = find(P_C1(3, :) <= 0);
P_C2 = [R T] * P_C1;
P_C2 = [P_C2; ones(1, size(P_C2, 2))];
outlier1_idx = find(P_C2(3, :) <= 0);

% Removing outliers
P_C1(:, outlier0_idx) = [];
P_C2(:, outlier0_idx) = [];
points0_h(:, outlier0_idx) = [];
points1_h(:, outlier0_idx) = [];

% Reproject Points to image planes
points0_h_reprojected = M1*P_C1;
points0_h_reprojected = points0_h_reprojected./points0_h_reprojected(3, :);
points1_h_reprojected = M2*P_C1;
points1_h_reprojected = points1_h_reprojected./points1_h_reprojected(3, :);


% Removing outliers (ones with error more than 1 pixel) in image 1
err0 = abs(points0_h_reprojected - points0_h);
outlier0_idx = find(err0(1, :) >= 1);
P_C1(:, outlier0_idx) = [];
P_C2(:, outlier0_idx) = [];
points0_h(:, outlier0_idx) = [];
points1_h(:, outlier0_idx) = [];
points0_h_reprojected(:, outlier0_idx) = [];
points1_h_reprojected(:, outlier0_idx) = [];

% Removing outliers (ones with error more than 1 pixel) in image 2
err1 = abs(points1_h_reprojected - points1_h);
outlier1_idx = find(err1(1, :) >= 1);
P_C1(:, outlier1_idx) = [];
P_C2(:, outlier1_idx) = [];
points0_h(:, outlier1_idx) = [];
points1_h(:, outlier1_idx) = [];
points0_h_reprojected(:, outlier1_idx) = [];
points1_h_reprojected(:, outlier1_idx) = [];

err0 = abs(points0_h_reprojected - points0_h);
err1 = abs(points1_h_reprojected - points1_h);



% Solving nonlinear least squares

n = size(P_C1, 2);
x0 = [double(R(:)); double(T); P_C1(:)];
Aeq = zeros(n, 12 + 4*n);
options = optimoptions('fmincon', 'Display', 'off');
for i=1:n
    Aeq(i, 16 + 4*(i-1)) = 1;
end
beq = ones(n, 1);
F = @(x)myfun(x, double(points0_h), double(points1_h), K1);
nonlcon = @mycon;
x = fmincon(F,x0,[],[],Aeq,beq,[],[], nonlcon, options);

% Extracting the rotation and translation
optimal_R = reshape(x(1:9), [3 3]);
optimal_T = x(10:12);
end










function F = myfun(x, points0_h, points1_h, K)
    n = size(points0_h, 2);
    M1 = K * eye(3,4);
    R = reshape(x(1:9), [3 3]);
    T = x(10:12);
    P = reshape(x(13:end), [4 n]);

    M2 = K * [R T];
    tmp1 = M1*P;
    tmp1 = tmp1./tmp1(3, :);
    tmp2 = M2*P;
    tmp2 = tmp2./tmp2(3, :);
    F = [points0_h(:) - tmp1(:); points1_h(:) - tmp2(:)];
    F = norm(F)^2;
end

function [c, ceq] = mycon(x)
    c = [];
    R = reshape(x(1:9), [3 3]);
    ceq(1:3, 1:3) = diag([(det(R) - 1), (det(R) - 1), (det(R) - 1)]);
    ceq(4:6, 1:3) = R*R' - eye(3);
    ceq(7:9, 1:3) = R' - inv(R);
end


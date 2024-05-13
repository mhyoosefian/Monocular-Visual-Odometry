function [optimal_R, optimal_T] = reprojectionErrorOptimization2(optimizationRot, optimizationTrans, selectedPointsForOptim, K)



numFrames = length(optimizationRot);
numFeatures = size(selectedPointsForOptim{end}, 1);
max_condition = 10000;

RCG = [];
pGC = [];
features = [];
for i=1:numFrames
    RCG = [RCG optimizationRot{i}];
    pGC = [pGC optimizationTrans{i}];
    features = [features selectedPointsForOptim{i}];
end

for i=1:numFrames
    M{i} = K * [optimizationRot{i} optimizationTrans{i}];
end

% Points in the world frame
for i=1:numFeatures
    [optimized_point, ~] = linear_triangulation(RCG, pGC, features(i, :), max_condition, K);
    P_G(:, i) = [-optimized_point; 1];
%     P_C{1}(:, i) = linearTriangulation([features(i, 1:2) 1]', [features(i, 3:4) 1]' , M{1}, M{2});
end
for i=1:numFrames
    for j=1:numFeatures
        tmp = [optimizationRot{i}, optimizationTrans{i}]*P_G(:, j);
        P_C{i}(:, j) = [tmp; 1];
    end
end


% Remove the points behind the camera
outlierIdx = find(P_G(3, :) <= 0);
for i=1:numFrames
    P_C{i}(:, outlierIdx) = [];
    selectedPointsForOptim{i}(outlierIdx, :) = [];
end
P_G(:, outlierIdx) = [];

% Show if all points are behind camera
if isempty(selectedPointsForOptim{1})
    disp('All points behind camera, no optimization');
end


% Remove the points that have abnormally large reprojection error
for i=1:numFrames

    % First, homogenize the features that we have seen in the images
    features_h{i} = [selectedPointsForOptim{i} ones(size(selectedPointsForOptim{i}, 1), 1)];

    % Now, compute the reprojected points
    features_reprojected_h{i} = (M{i}*P_C{1})';
    features_reprojected_h{i} = features_reprojected_h{i}./features_reprojected_h{i}(:, 3);

    % Now, compute the error
    err = abs(features_reprojected_h{i} - features_h{i});
    outlierIdx =  find(err(:, 1) >= 1);

    % Now, remove the outliers from all frames
    for j=1:numFrames
        P_C{j}(:, outlierIdx) = [];
        selectedPointsForOptim{j}(outlierIdx, :) = [];
    end

    P_G(:, outlierIdx) = [];
end


% Now, solve the optimization problem
% First, homogenize the features that we have seen in the images
numFeatures = size(selectedPointsForOptim{end}, 1);
features_h = {};
for i=1:numFrames
    features_h{i} = [selectedPointsForOptim{i} ones(size(selectedPointsForOptim{i}, 1), 1)];
end

% Solving nonlinear least squares
x0 = [];
for i=1:numFrames
    x0 = [x0; double(optimizationRot{i}(:))];
end
for i=1:numFrames
    x0 = [x0; double(optimizationTrans{i}(:))];
end
x0 = [x0; P_G(:)];

Aeq = zeros(numFeatures, 12*numFrames + 4*numFeatures);
options = optimoptions('fmincon', 'Display', 'off');
for i=1:numFeatures
    Aeq(i, 12*numFrames + 4*i) = 1;
end
beq = ones(numFeatures, 1);
F = @(x)myfun(x, features_h, K, numFeatures, numFrames);
nonlcon = @mycon;
x = fmincon(F,x0,[],[],Aeq,beq,[],[], nonlcon, options);
disp('Optimized!')

% Extracting the rotation and translation
for i=1:numFrames
    optimal_R{i} = reshape(x(9*i-8:9*i), [3 3]);
    optimal_T{i} = x(9*numFrames+3*i-2:9*numFrames+3*i);
end
optimal_P_G = reshape(x(12*numFrames+1:end), [4 numFeatures]);

end




% Additional Functions
function F = myfun(x, features_h, K, numFeatures, numFrames)
    
    for i=1:numFrames
        R{i} = reshape(x(9*i-8:9*i), [3 3]);
        T{i} = x(9*numFrames+ 3*i-2:9*numFrames+3*i);
        M{i} = K * [R{i} T{i}];
    end
    P_G = reshape(x(12*numFrames+1:end), [4 numFeatures]);
    
    F = [];
    for i=1:numFrames
        tmp = (M{i}*P_G)';
        tmp = tmp./tmp(:, 3);
        tmp = double(features_h{i}') - tmp';
        F = [F; tmp(:)];
    end
    F = norm(F)^2;
end

function [c, ceq] = mycon(x)
    c = [];
    ceq = [];
%     R = reshape(x(1:9), [3 3]);
%     ceq(1:3, 1:3) = diag([(det(R) - 1), (det(R) - 1), (det(R) - 1)]);
%     ceq(4:6, 1:3) = R*R' - eye(3);
%     ceq(7:9, 1:3) = R' - inv(R);
end

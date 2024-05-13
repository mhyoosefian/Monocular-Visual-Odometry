clear all
clc
addpath utils/
%%
ImgNames = dir('../mav0/cam0/data/*.png');
imageIdx = 1000;
focalLength = [458.654 457.296];
principalPoint = [367.215 248.375];
imageSize = [480 752];
K = [458.654 0 367.215
     0 457.296 248.375
     0       0       1];
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize, 'RadialDistortion',[-0.28340811, 0.07395907],...
             'TangentialDistortion', [0.00019359, 1.76187114e-05]);

GT = load('GT.mat');
grt_data = GT.data;
grt_data = grt_data(6 + 10*(imageIdx-23):10:end,:);
grt_quat = grt_data(:,5:8);     % this is "qGI"
grt_pos = grt_data(:,2:4);
RIC = [0.014865542981800  -0.999880929698000   0.004140296794220
    0.999557249008000   0.014967213324700   0.025715529948000
    -0.025774436697400   0.003756188357970   0.999660727178000];
pIC = [-0.0216401454975; -0.064676986768; 0.00981073058949];

Rot = eye(3);
Trans = zeros(3,1);

%%
n_max = 3659;
n = n_max;
minNumOfFeatures = 50;   %60
finalTr = Trans;
finalRot = Rot;
Features = {};
k = 0;
optimizationRot = {};
optimizationTrans = {};
for i=imageIdx:n
    tic;
    imgName = strcat('../mav0/cam0/data/',ImgNames(i).name);
    disp(['Percentage completed: ', num2str(100*(i-imageIdx)/n), '%']);
    img = imread(imgName);
    img = undistortImage(img, intrinsics);
    k = k + 1;
    Img(:,:,k) = img;
    [selectedPointsForOptim, Features, statusCode, relativeOrientation, relativeLocation] = processFrames2(Img, Features, minNumOfFeatures, K);
    R = relativeOrientation;     % this is "RCiC1"
    tr = relativeLocation.';     % this is "pC1Ci"
    Trans = Trans + Rot.' * tr;  % this is "pGCi"
    Rot = R * Rot;               % this is "RCiG"
    resultTr(:,i-imageIdx+1) = tr;
    resultRot(:,:,i-imageIdx+1) = R;
    finalRot(:,:,i-imageIdx+1) = Rot;
    finalTr(:,i-imageIdx+1) = Trans;
    optimizationRot{end+1} = Rot;
    optimizationTrans{end+1} = Trans;

    % Nonlinear optimization
    if statusCode == 3
        [optimal_R, optimal_T] = reprojectionErrorOptimization2(optimizationRot, optimizationTrans, selectedPointsForOptim, K);
        l = length(optimizationTrans);
        for k=1:l
            finalRot(:,:,i-imageIdx+1 - l + k) = optimal_R{k};
            finalTr(:,i-imageIdx+1 - l + k) = optimal_T{k};
        end
        optimizationRot = {finalRot(:,:,i-imageIdx+1)};
        optimizationTrans = {finalTr(:,i-imageIdx+1)};
    end

    if size(Img,3) == 2
        k = 1;
        Img(:,:,1) = Img(:, :, 2); % replace the previous camera measurement with the current one in order to be used in the next loop
    end
    iter_time(i) = toc;
    clc;
end

save('finalRot_Optimized.mat','finalRot');
save('finalTrans_Optimized.mat','finalTr');
save('runTime_Optimized.mat','iter_time');
rms_error = plotResult(finalRot, grt_data, finalTr);
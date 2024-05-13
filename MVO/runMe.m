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
finalTr = Trans;
finalRot = Rot;
prevImgName = strcat('../mav0/cam0/data/',ImgNames(imageIdx).name);
prevImg = imread(prevImgName);
prevImg = undistortImage(prevImg, intrinsics);
for i=imageIdx:n
    tic;
    disp(['Percentage completed: ', num2str(100*(i-imageIdx)/n), '%']);
    currImgName = strcat('../mav0/cam0/data/',ImgNames(i+1).name);
    currImg = imread(currImgName);
    currImg = undistortImage(currImg, intrinsics);
    [relativeOrientation,relativeLocation] = processFrames(prevImg, currImg, intrinsics, K);
    R = relativeOrientation;     % this is "RCiC1"
    tr = relativeLocation.';     % this is "pC1Ci"
    Trans = Trans + Rot.' * tr;  % this is "pGCi"
    Rot = R * Rot;               % this is "RCiG"
    resultTr(:,i-imageIdx+1) = tr;
    resultRot(:,:,i-imageIdx+1) = R;
    finalRot(:,:,i-imageIdx+2) = Rot;
    finalTr(:,i-imageIdx+2) = Trans;
    prevImg = currImg;
    iter_time(i) = toc;
    clc
end
save('finalRot_2d.mat','finalRot');
save('finalTrans_2d.mat','finalTr');
save('runTime_2d.mat','iter_time');
rms_error = plotResult(finalRot, grt_data, finalTr);
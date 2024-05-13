function [selectedPointsForUpdate, Features, statusCode, R, T] = processFrames2(Img, Features, minNumOfFeatures, K)


if isempty(Features)        % for the first image, the Features struct is empty
    isFirstImg = 1;
else
    isFirstImg = 0;
end

imageSize = size(Img(:,:,1));
% num_keypoints = 400;
% n = 350;
num_keypoints = 1000;
n = 800;


if isFirstImg
    points = detectHarrisFeatures(Img(:,:,end), 'MinQuality', 0.01);
    points = points.selectStrongest(num_keypoints);
    points = selectUniform(points,n,imageSize);
    pointsImg = double(points.Location);
    Features{1} = pointsImg;    
    selectedPointsForUpdate = Features;
    statusCode = 0;     % means we don't need an update
    R = eye(3);
    T = zeros(1, 3);
end

if ~isFirstImg
    Features{end+1} = [];
    pointTracker = vision.PointTracker('MaxBidirectionalError', 0.1, 'BlockSize', [21 21], 'MaxIterations', 50);      % track the detected features in previous frame
    pointsPrevImg = Features{end-1}(Features{end-1}(:,1)~=-1,:);
    initialize(pointTracker,pointsPrevImg,Img(:,:,1));
    [pointsNextImg,point_validity,scores] = pointTracker(Img(:,:,end));
    inlierIdx = ones(size(scores));
    for k=1:length(scores)                                              % features that are not observed anymore are
        if point_validity(k) ~= 1 %|| inlierIdx(k) ~= 1                  % a "-1" to be easily detected and stored as
            Features{end}(k,1:2) = [-1 -1];                             % selectedPointsForUpdate
        else
            Features{end}(k,1:2) = pointsNextImg(k,:);
        end 
    end
    
    % select points that are not observed in the current image to do an update
    selectedPointsForUpdate = [];
    for i=1:length(Features)-1
        Features{i}(Features{end}(:,1)==-1,:) = [];
        statusCode = 1; % we need an update but we don't need the state to be pruned
    end
    Features{end}(Features{end}(:,1) == -1,:) = [];

    % Estimate the relative pose between the two images
    [R, T, inlierIdx] = EstimateRelativePose(Features{end-1}, Features{end}, K);
    
    % Remove outliers given by RANSAC
    for k=1:length(inlierIdx)                                              % features that are not observed anymore are
        if inlierIdx(k) ~= 1                  % a "-1" to be easily detected and stored as
            Features{end}(k,1:2) = [-1 -1];                             % selectedPointsForUpdate
        end
    end
    for i=1:length(Features)-1
        Features{i}(Features{end}(:,1)==-1,:) = [];
        statusCode = 1; % we need an update but we don't need the state to be pruned
    end
    Features{end}(Features{end}(:,1) == -1,:) = [];

    
    % if the number of tracked features is less than a treshold, we should
    % do an update and detect a new set of keypoints (in the most recent image) to track

    len = length(Features{end});
    if len < minNumOfFeatures
        
        % detect new features in the current frame to be tracked, and use
        % the old features to do an optimization
        selectedPointsForUpdate = Features;
        points = detectHarrisFeatures(Img(:,:,end), 'MinQuality', 0.01);
        points = points.selectStrongest(num_keypoints);
        points = selectUniform(points,n,imageSize);
        pointsImg = double(points.Location);
        Features = {};
        Features{1} = pointsImg;
        statusCode = 3; % means the state and the covariance should be reset to IMU+the most recent image
    end

end
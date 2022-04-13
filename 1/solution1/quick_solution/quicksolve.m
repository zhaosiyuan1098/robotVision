clear;

%load images and data processing
imgPath='.\images\';
squareSize=62;
imgPathList = dir(strcat(imgPath,'*.jpg'));
imgNum = length(imgPathList);

imageFileNames = {1,imgNum};
if imgNum>0
    for i = 1:imgNum
      imageFileNames{i} = sprintf('images/%d.jpg', i);
    end
end

[imagePoints,boardSize,imagesUsed4dete] = detectCheckerboardPoints(imageFileNames);
M = generateCheckerboardPoints(boardSize,squareSize)';

[params,imagesUsed4esti, estimationErrors] = estimateCameraParameters(imagePoints,M', ...
                                  'ImageSize',boardSize);
                              
                                                           
h1=figure; showReprojectionErrors(params);
% Visualize pattern locations
h2=figure; showExtrinsics(params, 'CameraCentric');
% Display parameter estimation errors
displayErrors(estimationErrors, params);
originalImage=imread(imageFileNames{2});
% undistortedImage = undistortImage(originalImage, params);
% figure(1)
% imshow(undistortedImage)
% hold on
% imshow(originalImage)                              
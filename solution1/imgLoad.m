imgPath='.\images\';
squareSize=0.01;
imgPathList = dir(strcat(imgPath,'*.jpg'));
imgNum = length(imgPathList);

imageFileNames = {1,imgNum};
if imgNum>0
    for i = 1:imgNum
      imageFileNames{i} = sprintf('images/%d.jpg', i);
    end
end

[imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(imageFileNames);
m=permute(imagePoints,[2 1 3]);
M = generateCheckerboardPoints(boardSize,squareSize)';



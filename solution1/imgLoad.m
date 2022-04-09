%load images and data processing
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

% 2.2 Homography between the model plane and its image
boardCornerNum=size(imagePoints,1);
rowAllOne=ones(1,boardCornerNum);
M=[M;rowAllOne];
for i =1:imgNum
    m(3,:,i)=rowAllOne;
end

% according to the appendix A of the paper,calculate H for each image
L = zeros(2*boardCornerNum,9);
for i = 1:imgNum
    H(:,:,i) = solveH(m(:,:,i),M);
end

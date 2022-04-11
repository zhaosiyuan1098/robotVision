clear;

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

%solve V (A summarized matrix to restrict B)
V = zeros(2*imgNum,6);
for i=1:imgNum
    [V(2*i-1,:),V(2*i,:)]=solveB(H(:,:,i));
end

%solve B
[u,s,v]=svd(V);
b = v(:,end);

%solve A(intrinsic matrix
v0    = (b(2)*b(4)-b(1)*b(5))/(b(1)*b(3)-b(2)^2);
lamda = b(6)-(b(4)^2+(b(2)*b(4)-b(1)*b(5))/(b(1)*b(3)-b(2)^2)*(b(2)*b(4)-b(1)*b(5)))/b(1);
alpha = sqrt(lamda/b(1));
beta  = sqrt((lamda*b(1))/(b(1)*b(3)-b(2)^2));
c     = -(b(2)*alpha^2*beta)/lamda;
u0    = (c*(b(2)*v0))/alpha-(b(4)*alpha^2)/lamda;
A = [alpha c u0;0 beta v0;0 0 1];

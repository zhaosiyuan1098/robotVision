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

%solve A(intrinsic matrix)
v0    = (b(2)*b(4)-b(1)*b(5))/(b(1)*b(3)-b(2)^2);
lamda = b(6)-(b(4)^2+(b(2)*b(4)-b(1)*b(5))/(b(1)*b(3)-b(2)^2)*(b(2)*b(4)-b(1)*b(5)))/b(1);
alpha = sqrt(lamda/b(1));
beta  = sqrt((lamda*b(1))/(b(1)*b(3)-b(2)^2));
c     = -(b(2)*alpha^2*beta)/lamda;
u0    = (c*(b(2)*v0))/alpha-(b(4)*alpha^2)/lamda;
A = [alpha c u0;0 beta v0;0 0 1];

%solve RT(extrinic matrix)
lamda = (norm(inv(A)*H(:,1))+norm(inv(A)*H(:,2)))/2;
r1 = 1/lamda*inv(A)*H(:,1);
r2 = 1/lamda*inv(A)*H(:,2);
r3 = cross(r1, r2);
R = [r1 r2 r3];
[u,s,v] = svd(R);
R = u*v';
t = 1/lamda*inv(A)*H(:,3);
RT = [R(:,1) R(:,2) t];

%solve Ks
D = [];
d = [];
for i = 1:imgNum
    RT = solveRT(A,H(:,:,i));
    XY = RT*M;
    UV = A*XY;
    XY=[XY(1,:)./XY(3,:); XY(2,:)./XY(3,:); XY(3,:)./XY(3,:)];
    UV=[UV(1,:)./UV(3,:); UV(2,:)./UV(3,:); UV(3,:)./UV(3,:)];
    for j = 1:boardCornerNum
        D = [D; [(UV(1,j)-u0)*( (XY(1,j))^2 + (XY(2,j))^2 ), (UV(1,j)-u0)*( (XY(1,j))^2 + (XY(2,j))^2 )^2] ;
                [(UV(2,j)-v0)*( (XY(1,j))^2 + (XY(2,j))^2 ), (UV(2,j)-v0)*( (XY(1,j))^2 + (XY(2,j))^2 )^2]];
        
        d = [d; (m(1,j,i)-UV(1,j)) ; (m(2,j,i)-UV(2,j))];
    end
end
k0 = inv(D'*D)*D'*d;
param3 = [alpha c u0 beta v0 k0'];
options = optimset('Algorithm','levenberg-marquardt');
[x,res3] = lsqnonlin(@toMinimize2, param3 ,[],[], options, m, M, H);
res3 = res3/(imgNum*boardCornerNum);
alpha = x(1);c = x(2);u0 = x(3);beta = x(4);v0 = x(5);
A3 = [alpha c u0;0 beta v0;0 0 1];
A = A3;
k = [x(6); x(7)];
disp("A:");
disp(A);
disp("K:");
disp(k);
disp("平均误差为：");
disp(res3);

clear;
% load images
imagePath='..\homework2\';
imageName='toMeasure.bmp';
fullfile(imagePath,imageName);
originImg=imread(fullfile(imagePath,imageName));
[rows, columns, channelNum] = size(originImg);
if channelNum > 1
	outputMessage="the number of channel is bigger than 3,convertion to gray will start";
    disp(outputMessage);
	grayImg = rgb2gray(originalImg);
else
    grayImg=originImg;
end

%compare four methods of edge detection
BW1=edge(grayImg,'Roberts',0.1);
BW2=edge(grayImg,'Sobel',0.03);
BW3=edge(grayImg,'Prewitt',0.09);
BW4=edge(grayImg,'Canny',0.2);
edgeImg=BW4;

% figure;
% subplot(2,2,1);
% imshow(BW1);
% title('Robert') 
% subplot(2,2,2);
% imshow(BW2);
% title('Sobel')
% subplot(2,2,3);
% imshow(BW3);
% title('Prewitt');
% subplot(2,2,4);
% imshow(BW4);
% title('Canny')

% [H,T,R] = hough(edgeImg,'Theta',-90:1:89);
% P=houghpeaks(H,10,'Threshold',0.2*max(H(:)));
% figure;
% subplot(2,1,1);
% imshow(imadjust(rescale(H)),'XData',T,'YData',R,...
%       'InitialMagnification','fit');
% title('Hough transform of gantrycrane.png');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% colormap(gca,hot);
% subplot(2,1,2);
% imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% plot(T(P(:,2)),R(P(:,1)),'s','color','white');

%find circles generally for next step
[centers, radii, metric] = imfindcircles(edgeImg,[20,80],'EdgeThreshold',0.02);
% figure;
% imshow(edgeImg);
viscircles(centers, radii,'EdgeColor','b');

%Eliminate the tiny unconnected parts of the picture
se = strel('disk',5);
edgeCloseImg=imclose(edgeImg,se);
% figure;
% imshow(edgeCloseImg);

roiImg=zeros(size(originImg));
for curCol=1:size(roiImg,1)
    for curRow=1:size(roiImg,2)
        for i =1:size(centers,1)
            dis=sqrt(abs((curCol-centers(i,1)))^2+abs((curRow-centers(i,2))^2));
            if  dis>radii(i)-3 && dis<radii(i)
            roiImg(curRow,curCol)=255;
            roiImg(curRow,curCol)=roiImg(curRow,curCol)&&edgeCloseImg(curRow,curCol);
            end
        end
    end
end

figure;
imshow(roiImg);
    

%detect the pixels of circles
[labelImg,num]=bwlabel(roiImg);
circleEdges={};
minEdgesLength=200;
maxEdgesLength=400;

for i=1:num
    Edge=[];
    [r,c]=find(labelImg==i);
    length(r)
    if length(r)>minEdgesLength && length(r)<maxEdgesLength
        Edge(end+1:end+length(r),1:2)=[r,c];
        circleEdges{end+1}=Edge;
    end
end

%resolution pixels above and calculate accurate circles' centers and
%radiums
figure;
imshow(originImg);
hold on;
for i=1:length(circleEdges)
    
    circle=circleEdges{i};
    subCircle=Subpixel_Extraction(circle,originImg);
    abc=[subCircle(:,2),subCircle(:,1),ones(size(subCircle,1),1)]\-(subCircle(:,2).^2+subCircle(:,1).^2);
    a=abc(1);b=abc(2);c=abc(3);
    xc = -a/2;
    yc = -b/2;
    rad = sqrt((xc^2 + yc^2) - c);
    center=[yc,xc];
    circle_theta=-pi:0.01:pi;
    xfit=rad*cos(circle_theta)+center(2);
    yfit=rad*sin(circle_theta)+center(1);
    plot(xfit,yfit,'b-');
    rad
%     pause;
end


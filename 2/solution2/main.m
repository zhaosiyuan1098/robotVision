clear;
%% load images
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

%% compare four methods of edge detection

BW1=edge(grayImg,'Roberts',0.1);
BW2=edge(grayImg,'Sobel',0.03);
BW3=edge(grayImg,'Prewitt',0.09);
BW4=edge(grayImg,'Canny',0.2);
edgeImg=BW4;

%% find circles roughlly for next step

[foreCenters, radii, metric] = imfindcircles(edgeImg,[20,80],'EdgeThreshold',0.02);

%Eliminate the tiny unconnected parts of the picture
se = strel('disk',5);
edgeCloseImg=imclose(edgeImg,se);

roiImg=zeros(size(originImg));
for curCol=1:size(roiImg,1)
    for curRow=1:size(roiImg,2)
        for i =1:size(foreCenters,1)
            dis=sqrt(abs((curCol-foreCenters(i,1)))^2+abs((curRow-foreCenters(i,2))^2));
            if  dis>radii(i)-3 && dis<radii(i)
                roiImg(curRow,curCol)=255;
                roiImg(curRow,curCol)=roiImg(curRow,curCol)&&edgeCloseImg(curRow,curCol);
            end
        end
    end
end

%% detect and save pixels of circles

[labelImg,num]=bwlabel(roiImg);
circleEdges={};
minEdgesLength=200;
maxEdgesLength=400;

for i=1:num
    Edge=[];
    [r,c]=find(labelImg==i);
    if length(r)>minEdgesLength && length(r)<maxEdgesLength
        Edge(end+1:end+length(r),1:2)=[r,c];
        circleEdges{end+1}=Edge;
    end
end
%% resolution pixels and calculate accurate circles

subCenters=zeros(size(foreCenters));
subRadiums=zeros(size(foreCenters,1),1);
for i=1:length(circleEdges)
    subCircle=Subpixel_Extraction(circleEdges{i},originImg);
    abc=[subCircle(:,2),subCircle(:,1),ones(size(subCircle,1),1)]\-(subCircle(:,2).^2+subCircle(:,1).^2);
    a=abc(1);b=abc(2);c=abc(3);
    xc = -a/2;
    yc = -b/2;
    subCenters(i,:)=[yc,xc];
    subRadiums(i)=sqrt((xc^2 + yc^2) - c);
    %     pause;
end

%% calculate distances in image coordinates of each circle centers

centersSize=length(subCenters);
distanceMatrix=zeros(centersSize);
for i =1:centersSize
    for j =i:centersSize
        distanceMatrix(i,j)=sqrt(abs(subCenters(i,1)-subCenters(j,1))^2+abs(subCenters(i,2)-subCenters(j,2))^2);
        distanceMatrix(j,i)=distanceMatrix(i,j);
    end
end

% distanceMatrix

%% identify ruler and calculate rate between pixels and turth

startCutPoint=[472,339];
endCutPoint=[817,355];

rulerOriginImg =imcrop(originImg,[startCutPoint(1),startCutPoint(2),abs(startCutPoint(1)-endCutPoint(1)),abs(startCutPoint(2)-endCutPoint(2))]);
rulerEdgeImg=edge(rulerOriginImg,'Prewitt',0.01);

[H,T,R] = hough(rulerEdgeImg,'Theta',-1:1);
P=houghpeaks(H,10,'Threshold',0.1*max(H(:)));
lines = houghlines(rulerEdgeImg,T,R,P,'FillGap',1,'MinLength',13);
startX=lines(1).point1(1);
endX=startX;
for k = 1:length(lines)
    if lines(k).point1(1)>endX
        endX=lines(k).point1(1);
    end
    if lines(k).point1(1)<endX
        startX=lines(k).point1(1);
    end
end

convRate=50/abs(endX-startX);

%% calculate real size
trueDistanceMatrix=distanceMatrix.*convRate;
trueSubDiameters=subRadiums.*convRate*2;


%% draw
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
% 
% figure;
% imshow(edgeImg);
% viscircles(foreCenters, radii,'EdgeColor','b');
% 
% figure;
% imshow(edgeCloseImg);
% 
% figure;
% imshow(roiImg);
% 
% figure;
% imshow(originImg);
% hold on;
% circleTheta=-pi:0.01:pi;
% for i =1:length(subCenters)
%     xfit=subRadiums(i)*cos(circleTheta)+subCenters(i,2);
%     yfit=subRadiums(i)*sin(circleTheta)+subCenters(i,1);
%     plot(xfit,yfit,'b-');
% end
% 
% figure;
% subplot(2,1,1);
% imshow(imadjust(rescale(H)),'XData',T,'YData',R,...
%     'InitialMagnification','fit');
% title('Hough transform of gantrycrane.png');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% colormap(gca,hot);
% subplot(2,1,2);
% imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% plot(T(P(:,2)),R(P(:,1)),'s','color','white');
% 
% figure;
% subplot(3,1,1);
% imshow(rulerOriginImg);
% subplot(3,1,2);
% imshow(rulerEdgeImg)
% subplot(3,1,3);
% imshow(rulerEdgeImg), hold on
% for k = 1:length(lines)
%     xy = [lines(k).point1; lines(k).point2];
%     plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%     
%     % Plot beginnings and ends of lines
%     plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%     plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% end


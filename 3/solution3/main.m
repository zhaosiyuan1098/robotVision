clear;
intrinsicMatrix=[1555.00918908222	,0	,0;
                0	,1574.28624772510,	0;
                963.674168044664,	628.130992891514,	1];
face_camera_Distance=1000;
videoReader = VideoReader('siyuanmove.mp4','CurrentTime',0);
faceDetector = vision.CascadeObjectDetector();
%%
faceInitialFlag=0;
while hasFrame(videoReader)
    frame=readFrame(videoReader);
%     sharpColorFrame = imsharpen(frame);
%     adjustColorFrame = imadjust(sharpColorFrame,[.1 .1 0; .6 .7 1],[]);
    faceBbox = faceDetector(frame);
    if(size(faceBbox,1)>1)
        [~,faceIndex]=max(faceBbox(:,3));
        frame = insertShape(frame, 'Rectangle',faceBbox(faceIndex,:),'Color','white','LineWidth',5);
        figure; imshow(frame); title('检测到的人脸');
        points = detectMinEigenFeatures(rgb2gray(frame),'ROI',faceBbox(faceIndex,:));

        harrisPoints=points.selectStrongest(300);
        figure, imshow(frame), hold on, title('人脸特征点');
        plot(harrisPoints);
        harrisPoints=harrisPoints.Location;
        faceBboxPoints=bbox2points(faceBbox(faceIndex, :));
        lastPoints=harrisPoints;
        lastFaceCenter=mean(faceBboxPoints);
        faceInitialFlag=1;
        break;
    end
end

%%
min=10000;max=-10000;
numm=0;
x_arr=[];
y_arr=[];
if faceInitialFlag==1
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
    initialize(pointTracker, harrisPoints, frame);
    videoPlayer  = vision.VideoPlayer('Position',...
        [100 100 [size(frame, 2), size(frame, 1)]+30]);
    oldPoints = harrisPoints;
    while hasFrame(videoReader)
        frame = readFrame(videoReader);
        [harrisPoints, isFound] = step(pointTracker, frame);
        trackablePoints = harrisPoints(isFound, :);
        oldInliers = lastPoints(isFound, :);
        if size(trackablePoints, 1) >= 2
            [xform, inlierFlag] = estimateGeometricTransform2D(...
                oldInliers, trackablePoints, 'similarity', 'MaxDistance', 4);
            oldInliers    = oldInliers(inlierFlag, :);
            trackablePoints = trackablePoints(inlierFlag, :);
            faceBboxPoints = transformPointsForward(xform, faceBboxPoints);
            faceCenter=mean(faceBboxPoints);
            bboxPolygon = reshape(faceBboxPoints', 1, []);
            frame = insertShape(frame, 'Polygon', bboxPolygon,'LineWidth', 2);
            frame = insertMarker(frame, trackablePoints, '+','Color', 'white');
            frame = insertMarker(frame, faceCenter, '+','Color', 'red','Size',10);
            v_x=(faceCenter(1)-lastFaceCenter(1))*intrinsicMatrix(1,1)*60/1000/face_camera_Distance;            
            v_y=(faceCenter(2)-lastFaceCenter(2))*intrinsicMatrix(2,2)*60/1000/face_camera_Distance;
            x_arr=[x_arr,v_x];
            y_arr=[y_arr,v_y];  
            if v_y<min
                min=v_y;
            end
            if v_y>max
                max=v_y;
            end
            X=sprintf('x方向速度为%f m/s,y方向速度为%f m/s',v_x,v_y);
            disp(X);
            numm=numm+1
            lastPoints = trackablePoints;
            lastFaceCenter=faceCenter;
            setPoints(pointTracker, lastPoints);
        end
        step(videoPlayer, frame);
    end  
else
    disp("视频中未检测到人脸")
end
% release(videoPlayer);

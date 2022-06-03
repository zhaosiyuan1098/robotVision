% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector();

% Read a video frame and run the face detector.
videoReader = VideoReader('11.mp4','CurrentTime',0);
frame      = readFrame(videoReader);
bbox            = step(faceDetector, frame);

% Draw the returned bounding box around the detected face.
frame = insertShape(frame, 'Rectangle', bbox);
figure; imshow(frame); title('Detected face');
faceBboxPoints = bbox2points(bbox(1, :));
points = detectMinEigenFeatures(rgb2gray(frame), 'ROI', bbox(14,:));

% Display the detected points.
figure, imshow(frame), hold on, title('Detected features');
plot(points);

pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Initialize the tracker with the initial point locations and the initial
% video frame.
points = points.Location;
initialize(pointTracker, points, frame);

videoPlayer  = vision.VideoPlayer('Position',...
    [100 100 [size(frame, 2), size(frame, 1)]+30]);

lastPoints = points;

while hasFrame(videoReader)
    % get the next frame
    frame = readFrame(videoReader);

    % Track the points. Note that some points may be lost.
    [points, isFound] = step(pointTracker, frame);
    trackablePoints = points(isFound, :);
    oldInliers = lastPoints(isFound, :);
    
    if size(trackablePoints, 1) >= 2 % need at least 2 points
        
        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, inlierFlag] = estimateGeometricTransform2D(...
            oldInliers, trackablePoints, 'similarity', 'MaxDistance', 4);
        oldInliers    = oldInliers(inlierFlag, :);
        trackablePoints = trackablePoints(inlierFlag, :);
        
        % Apply the transformation to the bounding box points
        faceBboxPoints = transformPointsForward(xform, faceBboxPoints);
                
        % Insert a bounding box around the object being tracked
        bboxPolygon = reshape(faceBboxPoints', 1, []);
        frame = insertShape(frame, 'Polygon', bboxPolygon, ...
            'LineWidth', 2);
                
        % Display tracked points
        frame = insertMarker(frame, trackablePoints, '+', ...
            'Color', 'white');       
        
        % Reset the points
        lastPoints = trackablePoints;
        setPoints(pointTracker, lastPoints);        
    end
    
    % Display the annotated video frame using the video player object
    step(videoPlayer, frame);
end

% Clean up
release(videoPlayer);
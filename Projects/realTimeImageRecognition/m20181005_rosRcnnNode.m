function rosRcnnNode()

    % Instructions for running the simulation: https://github.com/riplaboratory/Kanaloa/tree/master/Tutorials/SoftwareInstallation/RobotX-Simulation
    % roslaunch robotx_gazebo vmrc.launch
    % roslaunch robotx_gazebo usv_keydrive.launch
    % roslaunch wamv_gazebo rviz_vmrc.launch
    % ...THEN, you can run this code

    % Close figures, clear command window, initialize ROS
    close all;
    clc;
    rosshutdown;
    rosinit;

    clear all

    % Import RCNN 
    load('rcnn.mat');
    
    % Code variables
    height = 800;       % vertical resolution of camera (input images)
    width = 800;        % horizontal resolution of camera (input images)
    scale = 0.5;        % output image scaling factor (<1 to downscale, >1 to upscale)
    closeness = 10;     % filter that removes detections closer to each other than this value [pixels]

    % Initialize ROS publisher
    rcnnPub = rospublisher('/rcnn_front_left_camera/image_raw/','sensor_msgs/Image');
    
    % Initialize ROS callback subscriber
    cameraSub = rossubscriber('/front_left_camera/image_raw/compressed',@cameraCallback);
%     cameraSub = rossubscriber('/front_left_camera/image_raw',@cameraCallback);
           
    % Declare global variable for callback data storage
    global cameraImage
    
    % Pause (to ensure the first message has published)
    pause(0.5);

    % Preallocate data matrices for speed (images must remain the same resolution or the code will break)
    img = zeros(height,width,3,'uint8');
    img_resize = zeros(round(height*scale),round(width*scale),3,'uint8');
    
    while true
                
        % Convert compressed image in global variable (from callback) to uint8
        img = readImage(cameraImage);
                
        % Downscale image
        img_resize = imresize(img,scale);
        
        % Detect features on the image using RCNN
        % https://www.mathworks.com/help/vision/ref/fastrcnnobjectdetector.detect.html
        [bbox,score,label] = detect(rcnn,img_resize,...
            'Threshold',0.85,...
            'SelectStrongest',false,...
            'MaxSize',[40,40]);
        
        % Sort scores
        [row,~] = size(label);
        circleScores = zeros(row,1);
        cruciformScores = zeros(row,1);
        triangleScores = zeros(row,1);
        circleBbox = zeros(row,4);
        cruciformBbox = zeros(row,4);
        triangleBbox = zeros(row,4);
        for i = 1:1:row
            if label(i,1) == 'circle'
                circleScores(i,1) = score(i,1);
                circleBbox(i,:) = bbox(i,:);
            elseif label(i,1) == 'cruciform'
                cruciformScores(i,1) = score(i,1);
                cruciformBbox(i,:) = bbox(i,:);
            else
                triangleScores(i,1) = score(i,1);
                triangleBbox(i,:) = bbox(i,:);
            end
        end
        
        % Remove zero rows
        circleScores = circleScores(any(circleScores,2),:);
        cruciformScores = cruciformScores(any(cruciformScores,2),:);
        triangleScores = triangleScores(any(triangleScores,2),:);
        circleBbox = circleBbox(any(circleBbox,2),:);
        cruciformBbox = cruciformBbox(any(cruciformBbox,2),:);
        triangleBbox = triangleBbox(any(triangleBbox,2),:);
        
        % Combine similar bounding boxes
        [mCircleScores,mCircleBboxes] = mergeBbox(circleScores,circleBbox,img_resize);
        [mCruciformScores,mCruciformBboxes] = mergeBbox(cruciformScores,cruciformBbox,img_resize);
        [mTriangleScores,mTriangleBboxes] = mergeBbox(triangleScores,triangleBbox,img_resize);
        
        % Display result
        imgDetect = img_resize;
        try
            [row,~] = size(mCircleScores);
            for i = 1:1:row
                circleAnn = sprintf('o (%0.2f)',mCircleScores(i,:));
                imgDetect = insertObjectAnnotation(imgDetect,'rectangle',[mCircleBboxes(i,:)],{circleAnn});
            end
            [row,~] = size(mCruciformScores);
            for i = 1:1:row
                cruciformAnn = sprintf('+ (%0.2f)',mCruciformScores(i,:));
                imgDetect = insertObjectAnnotation(imgDetect,'rectangle',[mCruciformBboxes],{cruciformAnn});
            end
            [row,~] = size(mTriangleScores);
            for i = 1:1:row
                triangleAnn = sprintf('^ (%0.2f)',mTriangleScores(i,:));
                imgDetect = insertObjectAnnotation(imgDetect,'rectangle',[mTriangleBboxes],{triangleAnn});
            end
        end
                
        % Publish to topic
        msg = rosmessage(rcnnPub);
        msg.Encoding = 'rgb8';
        writeImage(msg,imgDetect);
        send(rcnnPub,msg);
        
        % Short pause
        pause(0.01);
        
    end

end

function [mergedScores,mergedBBoxes] = mergeBbox(scores,bboxes,I)
    % Thanks in large part to sample code from:
    % https://www.mathworks.com/help/vision/ref/bboxoverlapratio.html

    try

        % Convert from the [x y width height] bounding box format to the [xmin ymin
        % xmax ymax] format for convenience.
        xmin = bboxes(:,1);
        ymin = bboxes(:,2);
        xmax = xmin + bboxes(:,3) - 1;
        ymax = ymin + bboxes(:,4) - 1;

        % Clip the bounding boxes to be within the image bounds
        xmin = max(xmin, 1);
        ymin = max(ymin, 1);
        xmax = min(xmax, size(I,2));
        ymax = min(ymax, size(I,1));

        % Compute the overlap ratio
        expandedBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];
        overlapRatio = bboxOverlapRatio(expandedBBoxes, expandedBBoxes);

        % Set the overlap ratio between a bounding box and itself to zero to
        % simplify the graph representation.
        n = size(overlapRatio,1); 
        overlapRatio(1:n+1:n^2) = 0;

        % Create the graph
        g = graph(overlapRatio);

        % Find the connected text regions within the graph
        componentIndices = conncomp(g);
        
        % Merge scores based on mean
        mergedScores = accumarray(componentIndices',scores,[],@mean);
        
        % Merge the boxes based on the minimum and maximum dimensions.
        xmin = accumarray(componentIndices',xmin,[],@min);
        ymin = accumarray(componentIndices',ymin,[],@min);
        xmax = accumarray(componentIndices',xmax,[],@max);
        ymax = accumarray(componentIndices',ymax,[],@max);
        
        % Compose the merged bounding boxes using the [x y width height] format.
        mergedBBoxes = [xmin ymin xmax-xmin+1 ymax-ymin+1];
    
    catch
    
        mergedBBoxes = bboxes;
    
    end
    
end

function cameraCallback(~,message)
    
    % Declare global variables to store position and orientation
    global cameraImage
    
    % Set relevant message parameters to global variables
    cameraImage = message;
    assignin('base','cameraImage',cameraImage);
    
end
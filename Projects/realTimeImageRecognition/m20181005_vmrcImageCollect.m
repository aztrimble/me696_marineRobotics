function vmrcImageCollect()
% Collects images from a running vmrc simulation (or bag file of one).
 
% Instructions for running the simulation: https://github.com/riplaboratory/Kanaloa/tree/master/Tutorials/SoftwareInstallation/RobotX-Simulation
% roslaunch robotx_gazebo vmrc.launch
% roslaunch robotx_gazebo usv_keydrive.launch
% roslaunch wamv_gazebo rviz_vmrc.launch


    % Close figures, clear command window, initialize ROS
    close all;
    clc;
    rosshutdown;
    rosinit;
    
    % Code variables
    nImg = 100;         % total number of images to collect
    imgDelay = 1;       % delay in between image collection [s]
    height = 800;       % vertical resolution of camera (input images)
    width = 800;        % horizontal resolution of camera (inpult images)
    scale = 0.5;        % output image scaling factor (<1 to downscale, >1 to upscale)
    startCount = 663;   % number to start with when naming image sequence
    
    % Initialize ROS callback subscriber
    cameraSub = rossubscriber('/front_left_camera/image_raw/compressed',@cameraCallback);
    cameraSub2 = rossubscriber('/middle_right_camera/image_raw/compressed',@camera2Callback);
    
    % Declare global variable for callback data storage
    global cameraImage camera2Image

    % Pause (to ensure the first message has published)
    pause(1);

    % Preallocate data matrices for speed (images must remain the same resolution or the code will break)
    img = zeros(height,width,3,'uint8');
    img_resize = zeros(round(height*scale),round(width*scale),3,'uint8');
    img2 = zeros(height,width,3,'uint8');
    img2_resize = zeros(round(height*scale),round(width*scale),3,'uint8');
    
    for i = startCount:startCount+nImg-1
    
        % Convert compressed image to uint8
        img = readImage(cameraImage);
        img2 = readImage(camera2Image);
        
        % Downscale image
        img_resize = imresize(img,scale);
        img2_resize = imresize(img2,scale);

        % Write images to file
        imwrite(img_resize,sprintf('snap_%d-1.jpeg',i));
        imwrite(img2_resize,sprintf('snap_%d-2.jpeg',i));
        disp(['IMAGE TAKEN, PAUSING......',num2str(i)]);
    
        pause(imgDelay);

    end

end

function cameraCallback(~,message)
    
    % Declare global variables to store position and orientation
    global cameraImage
    
    % Set relevant message parameters to global variables
    cameraImage = message;
    
end

function camera2Callback(~,message)
    
    % Declare global variables to store position and orientation
    global camera2Image
    
    % Set relevant message parameters to global variables
    camera2Image = message;
    
end
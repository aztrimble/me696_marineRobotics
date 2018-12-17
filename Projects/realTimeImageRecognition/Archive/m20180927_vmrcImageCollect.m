function vmrcImageCollect()
% Collects images from a running vmrc simulation (or bag file of one).

    % Close figures, clear command window, initialize ROS
    close all;
    clc;
    rosshutdown;
    rosinit;
    
    % Code variables
    nImg = 100;           % total number of images to collect
    imgDelay = 1;       % delay in between image collection [s]
    height = 1080;      % vertical resolution of camera
    width = 1920;       % horizontal resolution of camera
    startCount = 1;     % number to start with when naming image sequence
    
    % Initialize ROS callback subscriber
    cameraSub = rossubscriber('/front_left_camera/image_raw/compressed',@cameraCallback);
    
    % Declare global variable for callback data storage
    global cameraImage

    % Pause (to ensure the first message has published)
    pause(1);

    % Preallocate data matrices for speed (images must remain the same resolution or the code will break)
    img = zeros(height,width,3,'uint8');
    
    for i = startCount:startCount+nImg-1
    
        % Convert compressed image to uint8
        img = readImage(cameraImage);

        % Write images to file
        imwrite(img,sprintf('snap_%d.png',i));
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

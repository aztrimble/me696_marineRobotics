% This script creates a training dataset, and trains a RCNN.
% You must have your images, and ground truth image labels (created by the
% "Image Labeler" toolbox) in the working directory.

% https://www.mathworks.com/help/vision/examples/object-detection-using-deep-learning.html
% https://www.mathworks.com/help/deeplearning/examples/visualize-activations-of-a-convolutional-neural-network.html;jsessionid=1b3e164b3b06614b417c646f5c10
% https://www.mathworks.com/matlabcentral/answers/396872-improper-initialization-of-classification-layer-in-rcnn
% https://www.mathworks.com/help/vision/ref/trainfasterrcnnobjectdetector.html
% https://www.mathworks.com/help/vision/ug/faster-r-cnn-basics.html
% https://www.mathworks.com/help/deeplearning/ug/pretrained-convolutional-neural-networks.html
% https://www.mathworks.com/help/releases/R2018b/vision/object-detection-using-deep-learning.html

% Create training dataset
groundTruth = load("groundTruth.mat");
trainingData = objectDetectorTrainingData(groundTruth.groundTruth);
  
% Image properties
height = 400;
width = 400;
categories = {'circle','cruciform','triangle'};
nCategories = numel(categories);

% Input layers
inputLayer = imageInputLayer([height,width,nCategories]);

% Hidden layers
filterSize = [5 5];
nFilters = 16;           % number of neuorons in the convolutional layer that connect to the same region in the input layer. Determines the number of channels (feature maps) in the output of the convolutional layer.
middleLayers = [
    convolution2dLayer(filterSize,nFilters)
    reluLayer()
%     maxPooling2dLayer(3,'Stride',2)
    convolution2dLayer(filterSize,nFilters)
    reluLayer()
%     maxPooling2dLayer(3,'Stride',2)
    convolution2dLayer(filterSize,nFilters)
    reluLayer()
    maxPooling2dLayer(3,'Stride',2)
    convolution2dLayer(filterSize,nFilters)
    reluLayer()
    maxPooling2dLayer(3,'Stride',2)
];

% Final layers
outputs = nCategories+1;            % +1 due to background class
finalLayers = [
    fullyConnectedLayer(64)
    reluLayer()
    fullyConnectedLayer(outputs)
    softmaxLayer()
    classificationLayer()
];

% Combine layers
layers = [
    inputLayer
    middleLayers
    finalLayers
    ];

% Train RCNN
doTraining = false;         % set this flag to true to train network
if doTraining == true

%     % Set options for training
%     options = trainingOptions('sgdm',...
%         'LearnRateSchedule','piecewise',...
%         'LearnRateDropFactor',0.1,...
%         'LearnRateDropPeriod',5,...
%         'MaxEpochs',50,...
%         'MiniBatchSize',128,...
%         'Verbose',true);

    options = trainingOptions('sgdm', ...
    'MiniBatchSize', 32, ...
    'InitialLearnRate', 1e-3, ...
    'MaxEpochs', 30, ...
    'Verbose',true);
    
    % Train RCNN
    rcnn = trainFastRCNNObjectDetector(trainingData,layers,options,...
        'NegativeOverlapRange', [0 0.3], 'PositiveOverlapRange',[0.5 1]);
    
else

    clear img
    % Test the network on a testng image
%     img = imread('snap_197-2.jpeg');
    img = imread('snap_69.jpeg');
%     img = imread('snap_84-1.jpeg');
%     img = imread('snap_29.jpeg');
    
    % Detect features on the image using RCNN
    % https://www.mathworks.com/help/vision/ref/fastrcnnobjectdetector.detect.html
    [bbox,score,label] = detect(rcnn,img,...
        'Threshold',0.01,...
        'SelectStrongest',false,...
        'MaxSize',[40,40]);

    % Sort scores
    [row,~] = size(label);
    circleScores = zeros(row,1);
    cruciformScores = zeros(row,1);
    triangleScores = zeros(row,1);
    for i = 1:1:row
        if label(i,1) == 'circle'
            circleScores(i,1) = score(i,1);
        elseif label(i,1) == 'cruciform'
            cruciformScores(i,1) = score(i,1);
        else
            triangleScores(i,1) = score(i,1);
        end
    end
        
    [circleScore,iCircle] = max(circleScores);
    [cruciformScore,iCruciform] = max(cruciformScores);
    [triangleScore,iTriangle] = max(triangleScores);

    % Display result
    bboxCircle = bbox(iCircle,:);
    bboxCruciform = bbox(iCruciform,:);
    bboxTriangle = bbox(iTriangle,:);
    circleAnn = sprintf('%s: (Confidence = %f)',label(iCircle),circleScore);
    cruciformAnn = sprintf('%s: (Confidence = %f)',label(iCruciform),cruciformScore);
    triangleAnn = sprintf('%s: (Confidence = %f)',label(iTriangle),triangleScore);
    circleDetect = insertObjectAnnotation(img,'rectangle',bboxCircle,circleAnn);
    cruciformDetect = insertObjectAnnotation(img,'rectangle',bboxCruciform,cruciformAnn);
    triangleDetect = insertObjectAnnotation(img,'rectangle',bboxTriangle,triangleAnn);
    allDetect = insertObjectAnnotation(img,'rectangle',[bboxCircle;bboxCruciform;bboxTriangle],{circleAnn,cruciformAnn,triangleAnn});
    figure(1);
    subplot(2,2,1);
    imshow(circleDetect);
    subplot(2,2,2);
    imshow(cruciformDetect);
    subplot(2,2,3);
    imshow(triangleDetect);
    subplot(2,2,4);
    imshow(allDetect);
    
end

% This script creates a training dataset, and trains a CNN.
% You must have your images, and ground truth image labels (created by the
% "Image Labeler" toolbox) in the working directory.

% https://www.mathworks.com/help/vision/examples/object-detection-using-deep-learning.html

% Create training dataset
groundTruth = load("groundTruth.mat");
trainingData = objectDetectorTrainingData(groundTruth.gTruth);

% Load images
imds = imageDatastore('~/Documents/MATLAB/ComputerVision/testImages/*.png');

% Image properties
height = 1080;
width = 1920;
nChannels = 3;
nImageCategories = 10;

% Create the image input layer for images
imageSize = [height width nChannels];
inputLayer = imageInputLayer(imageSize);

% Convolutional layer parameters
filterSize = [5 5];
nFilters = 32;

middleLayers = [
    % The first convolutional layer has a bank of 32 5x5x3 filters. A
    % symmetric padding of 2 pixels is added to ensure that image borders
    % are included in the processing. This is important to avoid
    % information at the borders being washed away too early in the
    % network.
    convolution2dLayer(filterSize, nFilters, 'Padding', 2)

    % Note that the third dimension of the filter can be omitted because it
    % is automatically deduced based on the connectivity of the network. In
    % this case because this layer follows the image layer, the third
    % dimension must be 3 to match the number of channels in the input
    % image.

    % Next add the ReLU layer:
    reluLayer()

    % Follow it with a max pooling layer that has a 3x3 spatial pooling area
    % and a stride of 2 pixels. This down-samples the data dimensions from
    % 32x32 to 15x15.
    maxPooling2dLayer(3, 'Stride', 2)

    % Repeat the 3 core layers to complete the middle of the network.
    convolution2dLayer(filterSize, nFilters, 'Padding', 2)
    reluLayer()
    maxPooling2dLayer(3, 'Stride',2)

    convolution2dLayer(filterSize, 2 * nFilters, 'Padding', 2)
    reluLayer()
    maxPooling2dLayer(3, 'Stride',2)
];

finalLayers = [
    % Add a fully connected layer with 64 output neurons. The output size of
    % this layer will be an array with a length of 64.
    fullyConnectedLayer(64)

    % Add an ReLU non-linearity.
    reluLayer

    % Add the last fully connected layer. At this point, the network must
    % produce 10 signals that can be used to measure whether the input image
    % belongs to one category or another. This measurement is made using the
    % subsequent loss layers.
    fullyConnectedLayer(nImageCategories)

    % Add the softmax loss layer and classification layer. The final layers use
    % the output of the fully connected layer to compute the categorical
    % probability distribution over the image classes. During the training
    % process, all the network weights are tuned to minimize the loss over this
    % categorical distribution.
    softmaxLayer
    classificationLayer
];

% Initialize the first convolutional layer weights using normally
% distributed random numbers with standard deviation of 0.0001. This helps
% improve the convergence of training.
layers(2).Weights = 0.0001 * randn([filterSize nChannels nFilters]);

% Set the network training options
opts = trainingOptions('sgdm', ...
    'Momentum', 0.9, ...
    'InitialLearnRate', 0.001, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropFactor', 0.1, ...
    'LearnRateDropPeriod', 8, ...
    'L2Regularization', 0.004, ...
    'MaxEpochs', 40, ...
    'MiniBatchSize', 128, ...
    'Verbose', true);

% A trained network is loaded from disk to save time when running the
% example. Set this flag to true to train the network.
doTraining = true;

if doTraining    

%     % Train a network.
%     network = trainNetwork(imds, groundTruth.gTruth.LabelData, layers, opts);

layers = [imageInputLayer([height width nChannels])
        convolution2dLayer([5 5],10)
        reluLayer()
        fullyConnectedLayer(10)
        softmaxLayer()
        classificationLayer()];
    
options = trainingOptions('sgdm',...
    'LearnRateSchedule','piecewise',...
    'LearnRateDropFactor',0.2,...
    'LearnRateDropPeriod',5,...
    'MaxEpochs',20,...
    'MiniBatchSize',64,...
    'Plots','training-progress');

    % https://www.mathworks.com/matlabcentral/answers/396872-improper-initialization-of-classification-layer-in-rcnn
    detector = trainRCNNObjectDetector(trainingData, layers, options);
        
else
    % Load pre-trained detector for the example.
    load('rcnnStopSigns.mat','cifar10Net')       
end

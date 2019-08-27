 clc; clear all;
 
 %%
 
 imds = imageDatastore('/home/pardon/Documents/1_grasp_actions_affordances/matlab_code/labelling_grasp_rectangles/cube/depth_images/train');
 %%
 srcFiles = dir('/home/pardon/Documents/1_grasp_actions_affordances/matlab_code/labelling_grasp_rectangles/cube/depth_images/train');  % the folder in which ur images exists
 
for i = 3 : length(srcFiles)
filename = strcat(srcFiles(i).name);
im = imread(filename);
k=imresize(im,[64,64]);
newfilename=strcat(srcFiles(i).name);
imwrite(k,newfilename,'png');
end
%%
 imageLabeler(imds)
 %%
 classNames = ["grasping_region_1" "grasping_region_2"  "background"];
 pixelLabelID = [1 2 3];
 
 pxDir = '/home/pardon/Documents/1_grasp_actions_affordances/matlab_code/labelling_grasp_rectangles/cube/PixelLabelData';

%%
 pxds = pixelLabelDatastore(pxDir,classNames,pixelLabelID);
 
 %% Checking the labels look OK
 C = readimage(pxds,1);
 
 I = readimage(imds,1);
 B = labeloverlay(I,C);
 figure
 imshow(B)
 
  %% Create the network to train the labels
 
I = read(imds);
C = read(pxds);
I = imresize(I,5);
L = imresize(uint8(C),5);
figure;imshowpair(I,L,'montage') %depthToCloud works with L
%% This one works
augmenter = imageDataAugmenter('RandXReflection',true);%,...
    %'RandXTranslation',[-10 10],'RandYTranslation',[-10 10]);

trainingData = pixelLabelImageDatastore(imds,pxds, ...
    'DataAugmentation',augmenter);
tbl = countEachLabel(trainingData);
totalNumberOfPixels = sum(tbl.PixelCount);
frequency = tbl.PixelCount / totalNumberOfPixels;
classWeights = 1./frequency;

numFilters = 64;
filterSize = 3;
numClasses = 3;
layers = [
    imageInputLayer([64 64])
    convolution2dLayer(filterSize,numFilters,'Padding',1)
    reluLayer()
    maxPooling2dLayer(2,'Stride',2)
    convolution2dLayer(filterSize,numFilters,'Padding',1)
    reluLayer()
    transposedConv2dLayer(4,numFilters,'Stride',2,'Cropping',1);
    convolution2dLayer(1,numClasses);
    softmaxLayer()
    pixelClassificationLayer()
    ];
layers(end) = pixelClassificationLayer('Classes',tbl.Name,'ClassWeights',classWeights);


opts = trainingOptions('sgdm', ...
    'LearnRateSchedule','piecewise',...
    'LearnRateDropPeriod',10,...
    'LearnRateDropFactor',0.3,...
    'Momentum',0.9, ...
    'InitialLearnRate',1e-3, ...
    'L2Regularization',0.005, ...    
    'MaxEpochs',30, ...  
    'MiniBatchSize',8, ...
    'Shuffle','every-epoch', ...    
    'VerboseFrequency',2,...
    'Plots','training-progress',...
    'ValidationPatience', 4);

net = trainNetwork(trainingData,layers,opts);
%%
testImage = imresize(imread('notebook_2_2_1_depthcrop.png'), [64 64]);
C = semanticseg(testImage,net);

B = labeloverlay(testImage,C,'IncludedLabels',"background",...
    'Colormap','autumn','Transparency',0.25);
figure;imshow(B)
g1 = labeloverlay(testImage,C,'IncludedLabels',"grasping_region_1",...
    'Colormap','autumn','Transparency',0.25);
figure;imshow(g1)
g2 = labeloverlay(testImage,C,'IncludedLabels',"grasping_region_2",...
    'Colormap','autumn','Transparency',0.25);
figure;imshow(g2)
% g3 = labeloverlay(testImage,C,'IncludedLabels',"grasping_region_3",...
%     'Colormap','autumn','Transparency',0.25);
% figure;imshow(g3)



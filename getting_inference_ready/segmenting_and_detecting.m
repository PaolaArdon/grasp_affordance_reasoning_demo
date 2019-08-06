close all; clc; clear all;

objects = imread("objects.png");
[object_1, rect1] = imcrop(objects);
environment_im = imread("environment.png");

%Environment
environment = load('../environment_model');
ds = augmentedImageDatastore(environment.imageSize, environment_im, 'ColorPreprocessing', 'gray2rgb');
imageFeatures = activations(environment.net, ds, environment.featureLayer, 'OutputAs', 'columns');
environment_label = string(predict(environment.classifier,imageFeatures, 'ObservationsIn', 'columns'));

%Shape
shape = load('../shape_model');
ds = augmentedImageDatastore(shape.imageSize, object_1, 'ColorPreprocessing', 'gray2rgb');
imageFeatures = activations(shape.net, ds, shape.featureLayer, 'OutputAs', 'columns');
shape_label = string(predict(shape.classifier,imageFeatures, 'ObservationsIn', 'columns'));

%Material
material = load('../materials_model');
ds = augmentedImageDatastore(material.imageSize, object_1, 'ColorPreprocessing', 'gray2rgb');
imageFeatures = activations(material.net, ds, material.featureLayer, 'OutputAs', 'columns');
material_label = string(predict(material.classifier,imageFeatures, 'ObservationsIn', 'columns'));

%Texture
texture = load('../texture_model');
ds = augmentedImageDatastore(texture.imageSize, object_1, 'ColorPreprocessing', 'gray2rgb');
imageFeatures = activations(texture.net, ds, texture.featureLayer, 'OutputAs', 'columns');
texture_label = string(predict(texture.classifier,imageFeatures, 'ObservationsIn', 'columns'));

%Category
categorical = load('../categorical_model');
ds = augmentedImageDatastore(categorical.imageSize, object_1, 'ColorPreprocessing', 'gray2rgb');
imageFeatures = activations(categorical.net, ds, categorical.featureLayer, 'OutputAs', 'columns');
category_label = string(predict(categorical.classifier,imageFeatures, 'ObservationsIn', 'columns'));

%generate test db for KB
fileID = fopen('inference_data.txt','wt');

hasShape = strcat('hasShape(x,',string(shape_label),')');
hasMaterial = strcat('hasMaterial(x,',string(material_label),')');
hasTexture = strcat('hasTexture(x,',string(texture_label),')');
isA = strcat('isA(x,',string(category_label),')');
canBeFound =  string(strcat('canBeFound(x,',string(environment_label),')'));

fprintf(fileID,'%s\n', ...
                hasShape, ...
                hasMaterial, ...
                hasTexture, ...
                isA, ...
                canBeFound);
fclose(fileID);

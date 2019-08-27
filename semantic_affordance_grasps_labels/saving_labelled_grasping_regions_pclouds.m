clc; clear all
%%
net = cylindrical_net; %change net gfor the shape model detected by the attributes
testImage = imresize(imread('bowl_1_1_1_depthcrop.png'), [64 64]);
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

I = imresize(testImage,5);
L = imresize(uint16(C),5);
figure;imshowpair(I,L,'montage')

%%
rgb=imresize(imread('bowl_1_1_1_crop.png'),[64 64]);
detector = vision.ForegroundDetector;
foregroundMask = detector(rgb);
%foregroundMask = ~foregroundMask;

figure; imshow(foregroundMask);

g2(~foregroundMask)= 0; %B(~mask)=0
figure;imshow(B);
%%
rgb=imresize(imread('bowl_1_1_1_crop.png'),[64 64]);
Igray = rgb2gray(rgb);
BW = imbinarize(Igray);
%BW = imclearborder(BW);
%BW = imfill(BW,'hole');
%BW = bwareafilt(BW,1);
figure
imshow(BW)

g1(~BW) = 0;
figure; imshow(g1);
%%
%loc = load('ball_1_1_2_loc.txt');
loc = [1 1];
depth_ori = imread('bowl_1_1_1_depthcrop.png');

pcloud = depthToCloud(depth_ori,loc);

rgb=imresize(g1,size(depth_ori));

figure;

p1 = pcloud(:,:,1); p1 = p1(:);
p2 = pcloud(:,:,2); p2 = p2(:);
p3 = pcloud(:,:,3); p3 = p3(:);

r = rgb(:,:,1); r = r(:);
g = rgb(:,:,2); g = g(:);
b = rgb(:,:,3); b = b(:);

col = double([r g b])/255;

% data subsampling
idx = 1:2:length(r);
p1 = p1(idx);
p2 = p2(idx);
p3 = p3(idx);

col = col(idx,:);
scatter3(p1,p2,p3,5,col);

bowl = pointCloud([p1 p2 p3],'Color',col);
pcwrite(bowl, 'bowl.ply');

hold on

for i=1:length(p1)
   if isnan(p3(i)), continue, end

   plot3(p1(i),p2(i),p3(i),'color',col(i,:));
end
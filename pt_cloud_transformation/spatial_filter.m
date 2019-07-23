close all; clear; clc

%cloud = pcread('ei_cpseg.pcd');
cloud = pcread('ei.pcd');

x = cloud.Location(:, :, 1);
y = cloud.Location(:, :, 2);
z = cloud.Location(:, :, 3);

x_idx = intersect(find(x > 0.7), find(x < 2));
y_idx = intersect(find(y > -0.5), find(y < 0.5));
z_idx = intersect(find(z > 0.2), find(z < 1.5));
total_idx = intersect(intersect(x_idx, y_idx), z_idx);

x_filtered = nan * ones(1080, 1920);
y_filtered = nan * ones(1080, 1920);
z_filtered = nan * ones(1080, 1920);

x_filtered(total_idx)=x(total_idx);
y_filtered(total_idx)=y(total_idx);
z_filtered(total_idx)=z(total_idx);

pcloud(:, :, 1) = reshape(x_filtered, 1080, 1920);
pcloud(:, :, 2) = reshape(y_filtered, 1080, 1920);
pcloud(:, :, 3) = reshape(z_filtered, 1080, 1920);

filtered = pointCloud(single(pcloud));
filtered.Color = cloud.Color;

pcshow(filtered)
pcwrite(filtered, 'ei_cropped.pcd')
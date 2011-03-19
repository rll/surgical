close all

shift = [0 0 0]';
xrange = 0:.1:500;
yrange = 0:.1:100;
zrange = 0:.1:100;

WorldPoints = zeros(3,length(xrange)+length(yrange)+length(zrange));

ct = 1;
for i=xrange
    WorldPoints(:,ct) = [i; 0; 0] + shift;
    ct = ct+1;
end

for i=yrange
    WorldPoints(:,ct) = [0;i; 0] + shift;
    ct = ct+1;
end

for i=zrange
    WorldPoints(:,ct) = [0;0; i] + shift;
    ct = ct+1;
end

% figure; scatter32(WorldPoints)
%%
im = imread([directory '/data/2010Jun18/calib/extr3.tif']);
cam=cams{3};
% [imPoints] = project_points2(WorldPoints,cam.axis_angle,cam.trans,cam.foc,cam.principlePoint,cam.distortions,cam.skew);
% [imPoints2] = project_points2(X_ext,cam.axis_angle,cam.trans,cam.foc,cam.principlePoint,cam.distortions,cam.skew);

[imPoints] = projectArray(WorldPoints,cam);
[imPoints2] = projectArray(X_ext,cam);

figure; imshow(im); hold on;
scatter2(imPoints)

figure; imshow(im); hold on;
scatter2(imPoints2)

% figure; imshow(imu); hold on;
% scatter2(x_ext)

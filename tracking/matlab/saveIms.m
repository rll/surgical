%% This script has two functions: 
%%  1) Calculate the bounding trapezoid of the band. The bounding trapezoid
%%  won't be a tight fit, but will have some padding around it.
%%  2) Save an appropriate subimage to disk.

%% Need the global variable SHAFT_DIRS and MODEL_ORIGINS from running reconstructPoints.m

global SHAFT_DIRS
vects = SHAFT_DIRS;
global MODEL_ORIGINS;
translations = MODEL_ORIGINS;

corners = [-5 0 -5;
           -5 0 5;
           10.5 0 5;
           10.5 0 -5];
corners = corners';
   
close all;
allcorners = zeros(2*length(nums), 4);

directory = HomeDirectory;
imDir = 'data/2010Jul01/rect/';
pre = 'test';
k = 1;
for i=1:length(nums)
    E = zeros(3,4);
    E(:,1) = vects(:,i);
    E(:,2) = [0 0 1]';
    E(:,3) = [0 -1 0]';
    E(:,4) = translations(:,i);
    
    corners_cam = E*[corners; ones(1,size(corners,2))];

    corners_imEquiv = K*corners_cam;
    corners_im = corners_imEquiv(1:2, :) ./ ([1;1]*corners_imEquiv(3,:));

    
    x0 = floor(min(corners_im(1,:)));
    x1 = ceil(max(corners_im(1,:)));
    y0 = floor(min(corners_im(2,:)));
    y1 = ceil(max(corners_im(2,:)));

	imNum = nums(i);
    im = imread([directory '/' imDir '/' pre int2str(k) '-_rect' int2str(imNum) '.tif']);

    cropped = im(y0:y1, x0:x1);
    
    corners_cropped = vadd(corners_im, - [x0;y0]);
    allcorners((2*i-1):(2*i),:) = corners_cropped;

    outfile = [directory '/' imDir '/BandOnly/' pre  int2str(k) '-' int2str(imNum) '.ppm']
%     imwrite(cropped*3, outfile, 'PPM');

    if mod(i,10) == 1
        figure; imshow(cropped*2); hold on;
        scatter(corners_cropped(1,:), corners_cropped(2,:), 'g');
    end
end

outcorners = zeros(length(nums), 8);
outcorners(:, 1:2:end) = allcorners(1:2:end,:);
outcorners(:, 2:2:end) = allcorners(2:2:end, :);

% save outcorners bandCorners.txt


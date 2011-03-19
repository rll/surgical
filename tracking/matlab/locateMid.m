%% Finds a place in the middle of the shaft where the gradient changes from
%% dark to light. 
% given: ex_loc, ex_angle, ex_rho
% ex_loc should be the point we expect to return
% ex_angle is the initial estimate for the angle of the sleeves/shaft with
%  the y-axis
% ex_rho is the estimate of the perpendicular distance from the midline to
%  the image origin (see Wikipedia page for Hough transform)

%% initial vars
pre = 'knottie';
nx = 1280;
ny = 960;
maskSide = 350;

% this is the expected grayscale value of the black shaft, after the image
% is smoothed. For some reason convolution scales the entire image down.
ex_gray = .1;           
% how much variation to allow before we rule out a color as belonging to
% the shaft
gray_tol = .15;

% We'll allow lines that are within this much of ex_rho
rho_tol = 200;
% Allow lines within this many degrees of ex_angle.
angle_tol = 20;
% expected width of shaft in image. 
armw_min = 60;
armw_max = 80;
% when finding parallel lines, allow to differ in angle by these many
% degrees.
angd_tol = 1;

% base tracking directory
directory = RemoteHomeDirectory;

% where the images are kept
imDir = 'data/2010Jul23/gray';
filename = [pre int2str(k) '-' int2str(imNum) '.tif']
im = imread([directory '/' imDir '/' filename]);

% grayscale image if not already 
if length(size(im)) == 3
    img = rgb2gray(im);
else
    img = im;
end

cannythresh = [.05, .1];

[bw, th] = edge(img, 'canny', cannythresh, 2);
figure; imshow(bw);

%% Do Hough transform and choose the most probable lines
%% allpeaks is an nx2 array, wherein thetas(allpeaks(i,2)) is the angle of
%% line i and rhos(allpeaks(i,1)) is the perpendicular distance to the
%% origin of line i. 

% angle discretization
T = -90:.1:89;
% do Hough transform
[H,thetas,rhos] = hough(bw, 'RhoResolution', .5, 'Theta', T);

% thresh = 0.20*max(H(:));
% how many votes a line needs to have in order to be considered 
thresh = 55;
% suppression neighborhood: if a line is taken to be a peak, ignore 
nhood = [13,19];
npeaks = 20;
minlen = 110;
% find lines that have sufficiently many votes.
allpeaks = houghpeaks(H, npeaks, 'Threshold', thresh, 'nhoodsize', nhood);

P = allpeaks;
T = thetas;
figure; imshow(bw, opt, 50); hold on;
showhoughpeaks

%% filter peaks
%% Choose lines that are close to the expected angle, and whose
%% perpendicular distances are close to expected.
%% "pekes" contains the filtered peaks.

bool = angleDiffBlind(ex_angle, thetas(allpeaks(:,2)), 'deg') < angle_tol ...
    & abs(rhos(allpeaks(:,1)) - ex_rho) < rho_tol;

pekes = allpeaks(bool, :);
P = pekes;
figure; imshow(bw, opt, 50); hold on;
showhoughpeaks

%% find parallel lines the right width apart. 
%% pekes1(i,:) and pekes2(i,:) should parametrize a pair of parallel lines

Ti = pekes(:,2);
angdMatrix = zeros(length(Ti));
distMatrix = zeros(length(Ti));

for i=1:length(Ti)
    for j=(i+1):length(Ti)
        d = angleDiff(T(Ti(i)), T(Ti(j)), 'deg');
        angdMatrix(i,j) = d;
        angdMatrix(j,i) = d;
    end
end

rhoi = pekes(:,1);
for i=1:length(Ti)
    for j=(i+1):length(Ti)
        distMatrix(i,j) = abs(rhos(rhoi(i)) - rhos(rhoi(j)));
        distMatrix(j,i) = abs(rhos(rhoi(i)) - rhos(rhoi(j)));
    end
end

boolMatrix = (distMatrix < armw_max) & (distMatrix > armw_min) & (angdMatrix < angd_tol);
[ii, jj] = find(boolMatrix);
bb = ii < jj;
ii2 = ii(bb);
jj2 = jj(bb);
pekes1 = pekes(ii2,:);
pekes2 = pekes(jj2,:);

%% output
for i=1:length(ii2)
    P = [pekes1(i,:); pekes2(i,:)];
    figure; imshow(bw, opt, 50);
    showhoughpeaks
end

%% for each pair, find rho and theta of midline
xx = 1:nx;
rhomids = zeros(1,size(pekes1,1));
thmids = zeros(1,size(pekes1,1));
for i=1:size(pekes1, 1)
    pk1 = pekes1(i,:);
    pk2 = pekes2(i,:);
    r1 = rhos(pk1(1));
    r2 = rhos(pk2(1));
    rhomids(i) = .5*(r1+r2);    % average rhos
    thmids(i) = thetas(pk1(2)); % just take theta of line1; the idea is that the angles are so close this doesn't make a big difference
end

figure; imshow(bw); hold on;
for i=1:length(rhomids)
    m = -cot(thmids(i)*pi/180);
    b = rhomids(i)/sin(thmids(i)*pi/180);
    plot(1:nx, m*(1:nx) + b);
end

%% Put together of pool of candidate points. 
%% Candidate points must exceed the thresholds for directional similarity,
%% magnitude strength, and closeness of the color to the expected shaft
%% color. 
%%  Note: When calculating color similarity, use grayscale value of the point a
%%  few increments back on the midline, since the color before our chosen
%%  point must be the shaft color and the color after must be the band
%%  color.
%%
%%  Note 2: Grayscale values are taken from smoothed image, and so ex_gray
%%  should refer to the expected smoothed grayscale value. 

sig = 2; 
% note: the 1.5 multiplier is mostly for display purposes. The images tend
% to be dark.
imgsm = conv2(im2double(img)*1.5, fspecial('gaussian', [9,9],sig), 'same'); % smoothed image

dg = GaussianDerivative(sig);
dX = imfilter(im2double(img)*1.5, dg, sig, 'replicate', 'same'); 
dY = imfilter(im2double(img)*1.5, dg', sig, 'replicate', 'same'); 
dMags = sqrt(dX.*dX + dY.*dY);
imOris = atan2(dY, dX);
nconsider = 4;

dispim  = imgsm;

candidate_locs = [];
candMags = [];
candSims = [];
for i=1:length(rhomids)
    
    % get tX, the points on the midline. 
    s = sin(thmids(i)*pi/180);
    c = cos(thmids(i)*pi/180);
    R = [-s -c; c -s];
    
    dx = sin(thmids(i)*pi/180);
    sgn = dx/abs(dx);
    dy = cos(thmids(i)*pi/180);
    dx = sgn*dx;
    dy = -sgn*dy;
    
    nsteps = min(floor(nx/dx), floor(abs(ny/dy)));
    X = [2:nsteps-1; zeros(1,nsteps-2)];
    tX = R*X;
    
    icept = rhomids(i)/s;
    if icept >= 0
        tX = vadd(tX, [0; rhomids(i)/(s)]);
    else 
        tX = vadd(tX, [rhomids(i)/c;0]);
    end     
        
    % got the list of points to be looking at. Now compare their
    % values/gradients.
    inds = sub2ind(size(img), round(tX(2,:)), round(tX(1,:)));
%     dispim(inds) = 255;
    gradMags = dMags(inds);
    graddirs = [dX(inds) ;dY(inds)];
    graddirs = graddirs./([1;1]*normArray(graddirs));
    gradDirSims = (graddirs' * [dx; dy])'; 
    vals = imgsm(inds);
    
    % calculate magthresh to be a fraction of the max gradient (not
    % considering outliers);
    magOutlierThresh = mean(gradMags) + 2*std(gradMags);
    magthresh = .4*max(gradMags(gradMags < magOutlierThresh));
    gradMags(gradMags >= magOutlierThresh) = magOutlierThresh;

    simthresh = .9;
    figure; plot(gradMags); hold on
    hold on; plot(gradDirSims, 'g');
    plot([1, nsteps], magthresh*[   1 1], 'r');
    plot([1, nsteps], simthresh*[1 1], 'm');
    
    % figure out which points pass the thresholds, and take first
    % "nconsider." The reason for only considering the first few is that 
    % the later points tend to fall on the hand itself or some other 
    % object, and the algorithm has mistaken these for the right point. 
    % On the other hand, since the beginning of the midline tends to fall
    % in the middle of the sleeve, there aren't many strong gradients, and
    % so the first signals are the ones we want. 
    bool = gradMags > magthresh & gradDirSims > simthresh...
        & [true true true abs(vals(1:(end-3)) - ex_gray) < gray_tol];
    firstidx = find(bool, nconsider, 'first');
    
    candidate_locs = [candidate_locs, tX(:,firstidx)];
    candMags = [candMags gradMags(firstidx)];
    candSims = [candSims gradDirSims(firstidx)];
end

%% From the pool, choose one
%% Score by (dists-to-expected)/(gradient-magnitude). 
difs = vadd(candidate_locs, -ex_loc);
dists = normArray(difs);
[mn, mi] = min(dists./(candMags));

%% display
if true
    fig = figure; imshow(dispim, opt, 50); hold on;
    scatter(candidate_locs(1,:), candidate_locs(2,:), 'g');
    scatter(candidate_locs(1, mi), candidate_locs(2,mi), 'm');
end

% figure; imshow(dispim)

%% outputs
midpt = candidate_locs(:,mi);

pekes3 = unique([pekes1; pekes2], 'rows');
peak_thetas = [thetas(pekes3(:,2))];

% take care of some angle-wrapping business
peak_thetas(peak_thetas - ex_angle >= 90) = peak_thetas(peak_thetas - ex_angle >= 90) - 180;
peak_thetas(ex_angle - peak_thetas >= 90) = peak_thetas(ex_angle - peak_thetas >= 90) + 180;

avgAngle = mean(peak_thetas);
if avgAngle >= 90
    avgAngle = avgAngle - 180;
elseif avgAngle < -90
    avgAngle = avgAngle + 180;
end

avgRho = mean(rhomids);
thetaPeaks = [thetas(pekes1(:,2))' thetas(pekes2(:,2))'];
rhoPeaks= [rhos(pekes1(:,1))' rhos(pekes2(:,1))'];

%% other stuff
%% Prepare for next iteration.
ex_loc = midpt;
ex_angle = avgAngle;
ex_rho = avgRho;



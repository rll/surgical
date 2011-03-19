%% getHoughLines
% given: ex_loc, ex_angle, ex_rgb
%% begin body
pre = 'test';

nx = 1280;
ny = 960;
maskSide = 350;

ex_gray = .0328;
gray_tol = .04;
rgb_tol = [10 10 10];
rho_tol = 50;
angle_tol = 5;
armw_min = 40;
armw_max = 70;

directory = RemoteHomeDirectory;
imDir = 'data/2010Jul01';
filename = [pre int2str(k) '-' int2str(imNum) '.tif']
im = imread([directory '/' imDir '/' filename]);

if length(size(im)) == 3
    %img = rgb2gray(im);
    img = im(:,:,1);
else
    img = im;
end


if imNum >= 406
    cannythresh = [.02, .03];
else
    cannythresh = [.05, .1];
end
[bw, th] = edge(img, 'canny', cannythresh, 2.5);
% figure; imshow(bw);
%% find hough peaks
T = -90:.1:89;
[H,thetas,rhos] = hough(bw, 'RhoResolution', .5, 'Theta', T);

thresh = 0.20*max(H(:));
nhood = [13,19];
npeaks = 20;
minlen = 110;
allpeaks = houghpeaks(H, npeaks, 'Threshold', thresh, 'nhoodsize', nhood);

P = allpeaks;
T = thetas;
% figure; imshow(bw); hold on;
% showhoughpeaks

%% filter peaks
bool = angleDiffBlind(ex_angle, thetas(allpeaks(:,2)), 'deg') < angle_tol ...
    & abs(rhos(allpeaks(:,1)) - ex_rho) < rho_tol;

pekes = allpeaks(bool, :);
P = pekes;
% figure; imshow(bw); hold on;
% showhoughpeaks

%% find parallel lines the right width apart. 
Ti = pekes(:,2);
angdMatrix = zeros(length(Ti));
distMatrix = zeros(length(Ti));

for i=1:length(Ti)
    for j=(i+1):length(Ti)
        d = angleDiff(T(Ti(i)), T(Ti(j)));
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

boolMatrix = (distMatrix < armw_max) & (distMatrix > armw_min);
[ii, jj] = find(boolMatrix);
bb = ii < jj;
ii2 = ii(bb);
jj2 = jj(bb);
pekes1 = pekes(ii2,:);
pekes2 = pekes(jj2,:);

%% output
% for i=1:length(ii2)
%     P = [pekes1(i,:); pekes2(i,:)];
%     figure; imshow(bw, opt, 50);
%     showhoughpeaks
% end

%% for each pair, walk along them simultaneously until reaching the first
%% high gradient. 
sig = 2; 
imgsm = conv2(im2double(img), fspecial('gaussian', [9,9],sig), 'same');
gau = fspecial('gaussian', [1,9], sig);
dg = GaussianDerivative(sig);
dX = imfilter(im2double(img), dg, sig, 'replicate', 'same'); 
dY = imfilter(im2double(img), dg', sig, 'replicate', 'same'); 
dMags = sqrt(dX.*dX + dY.*dY);
imOris = atan2(dY, dX);

dispim  = imgsm*2;


%% for each midline, find top gradient(s)


candidate_locs = [];
candMags = [];
candSims = [];
for i=1:length(rhomids)
    s = sin(thmids(i)*pi/180);
    c = cos(thmids(i)*pi/180);
    R = [-s -c; c -s];
    
    dx = sin(thmids(i)*pi/180);
    sgn = dx/abs(dx);
    dy = cos(thmids(i)*pi/180);
    dx = sgn*dx;
    dy = sgn*dy;
    
    nsteps = floor(nx/dx);
    X = [2:nsteps-1; zeros(1,nsteps-2)];
    tX = R*X;
    tX = vadd(tX, [0; rhomids(i)/(s)]);
    
    % got the list of points to be looking at, good. Now compare their
    % values/gradients.
    inds = sub2ind(size(img), round(tX(2,:)), round(tX(1,:)));
%     dispim(inds) = 255;
    gradMags = dMags(inds);
    graddirs = [dX(inds) ;dY(inds)];
    graddirs = graddirs./([1;1]*normArray(graddirs));
    gradDirSims = (graddirs' * [-dx; dy])'; 
    vals = imgsm(inds);
    
    magOutlierThresh = mean(gradMags) + 2*std(gradMags);
    magthresh = .4*max(gradMags(gradMags < magOutlierThresh));
    gradMags(gradMags >= magOutlierThresh) = magOutlierThresh;

    simthresh = .9;
%     figure; plot(gradMags);
%     hold on; plot(gradDirSims, 'g');
%     plot([1, nsteps], magthresh*[1 1], 'r');
%     plot([1, nsteps], simthresh*[1 1], 'm');
    
    bool = gradMags > magthresh & gradDirSims > simthresh...
        & [true true true abs(vals(4:end) - ex_gray) < gray_tol];
    
    candidate_locs = [candidate_locs, tX(:,bool)];
    candMags = [candMags gradMags(bool)];
    candSims = [candSims gradDirSims(bool)];
end

difs = vadd(candidate_locs, -ex_loc);
dists = normArray(difs);


fig = figure; imshow(dispim); hold on;
scatter(candidate_locs(1,:), candidate_locs(2,:), 'g');

[mn, mi] = min(dists./(candMags.^.5));
scatter(candidate_locs(1, mi), candidate_locs(2,mi), 'm');

% figure; imshow(dispim)

%% outputs
midpt = candidate_locs(:,mi);

peak_thetas = [thetas(pekes1(:,2)) thetas(pekes2(:,2))];

peak_thetas(peak_thetas - ex_angle >= 90) = peak_thetas(peak_thetas - ex_angle >= 90) - 180;
peak_thetas(ex_angle - peak_thetas >= 90) = peak_thetas(ex_angle - peak_thetas >= 90) + 180;
avgAngle = mean(peak_thetas);
if avgAngle >= 90
    avgAngle = avgAngle - 180;
elseif avgAngle < -90
    avgAngle = avgAngle + 180;
end

avgRho = mean(rhomids);


%% other stuff
ex_loc = midpt;
ex_angle = avgAngle;
ex_rho = avgRho;
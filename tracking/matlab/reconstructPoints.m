%% Recover roll angles from monocular images. 
%% Or rather, set up the optimization that will recover roll angles. 
%%
%%  End results are the globak variables, below, to which reconstructScore
%% needs access, and an initial guess X0.
%%
%%  As an intermediate step, it also calculates the vector of all the arm
%% shafts in camera coordinates. These are stored in "vects" and later the
%% global variable SHAFT_DIRS.
%%
%%  Also see reconstructScore.m;
%% 

load band_high.txt;
load band_low.txt;
load base_high.txt;
load base_low.txt;

directory = HomeDirectory;
imDir = 'data/2010Jul01/';
load([directory '/' imDir 'Calib_Results.mat'])
fx = fc(1);
fy = fc(2);             % to do: x and y in the right order? 
cx = cc(1);
cy = cc(2);

pts1 = band_high;
pts2 = band_low;
vects = zeros(3, size(pts1,2));
innerShaftDiam = 8.3; %mm

% get direction shaft is pointing in
Origins_cam = zeros(3, size(pts1,2));
Zbands = zeros(1,size(pts1,2));
Zbases = zeros(1,size(pts1,2));

normMat = [1/fx 0 -cx/fx; 0 1/fy -cy/fy; 0 0 1];
for i=1:size(pts1, 2)
    d = norm(pts1(:,i)/fx - pts2(:,i)/fy);
    D = innerShaftDiam;
    Z = D/d;

    O_im = (pts1(:,i) + pts2(:,i))/2;
    O_nim = normMat * [O_im; 1]; % projection of origin on normalized image plane (distance 1 from the focal point)
    
    O_cam = [Z*O_nim]; %3D coordinates of the origin in camera coordinates
    Origins_cam(:, i)  = O_cam;
    Zband = Z;
    Zbands(i) = Zband;
    
    % get direction of vector. Tricky because base clicks weren't aligned. (Neither
    % were band clicks, it seems)
    band1_nim = normMat * [band_high(:,i);1];
    band2_nim = normMat * [band_low(:,i);1];
    
    base1_nim = normMat * [base_high(:,i);1];
    base2_nim = normMat * [base_low(:,i);1];
    abc1 = cross(band1_nim, base1_nim);     %normal to plane
    abc2 = cross(band2_nim, base2_nim);
    
    uvw = cross(abc1, abc2);                 % common vector
    vect_cam = uvw/norm(uvw);
    vects(:,i) = vect_cam;

    % back calc for reference
    basePt = O_cam - 450*vect_cam;
    Zbases(i) = basePt(3);
end


%% vects = x-axis of model frame.
% figure; imshow(im*4); hold on
 K = [fx 0 cx; 0 fy cy; 0 0 1];
 imptsEquiv = K*Origins_cam;
 impts = imptsEquiv(1:2, :) ./ ([1;1] * imptsEquiv(3,:));
% scatter(impts(1,:), impts(2,:)); 

%% set up variables
% also hand-initialized
init_pin1 = [9 0 4.15]';
init_pin2 = [9 0 -4.15]';
init_cornerA1 = [6 -2.1089 3.65]';
init_cornerA2 = [6 -2.1089 -3.65]';
init_cornerB1 = [6 2.1089 3.65]';
init_cornerB2 = [6 2.1089 -3.65]';
load init_angs                      % hand-initialized
init_pts = [init_pin1; init_pin2; init_cornerA1; init_cornerA2; init_cornerB1; init_cornerB2];

X0 = [init_angs; init_pts];

global SHAFT_DIRS
global MODEL_ORIGINS
global NUMS
global INTRINSIC_MAT
global MEASUREMENTS
global VALID

SHAFT_DIRS = vects;
MODEL_ORIGINS = Origins_cam;
NUMS = nums;                    % indices of the images labeled.
INTRINSIC_MAT = K;

load pin1.txt
load pin2.txt
load Corner_a1.txt
load Corner_a2.txt
load Corner_b1.txt
load Corner_b2.txt

bool1 = any(pin1 ~= 0);
bool2 = any(pin2 ~= 0);
bool3 = any(Corner_a1 ~= 0);
bool4 = any(Corner_a2 ~= 0);
bool5 = any(Corner_b1 ~= 0);
bool6 = any(Corner_b2 ~= 0);

MEASUREMENTS = {pin1, pin2, Corner_a1, Corner_a2, Corner_b1, Corner_b2};
VALID = {bool1, bool2, bool3, bool4, bool5, bool6};


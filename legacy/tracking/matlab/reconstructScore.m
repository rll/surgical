function scores = reconstructScore(X)

%%% Score function, given 
%%% a 3x6 guess for the 3D coordinates of the 6 model points 
%%% 4 params -- gross roll for the model transformation params

%%% We already have a list of projected correspondences. Require one or more per image 
%%% (we disregard images that have fewer). 

%%% The 6 transformation params plus the 3D coords will give predicted
%%% projections u_i, v_i, for i in 1 to 6. The score for the image will be
%%% the sum (u_i - u~_i)^2 + (v_i - v~i)^2 over all visible points i. 
%%% The total score will be the sum over all images. 

%%% Perhaps there should be some term to make sure the extrinsics don't
%%% vary too much from image to image? After all, the object should be
%%% pretty stable across time. 

%%% X is ordered as follows: 
%%% (gross rolls)_i for i from 1 to length(nums)
%%% (XYZ coordinates)_j for j from 1 to 6
%% put inputs into useful formats
global SHAFT_DIRS;
directions = SHAFT_DIRS;
global MODEL_ORIGINS;
translations = MODEL_ORIGINS;
global NUMS;
nums = NUMS;
global INTRINSIC_MAT;
K = INTRINSIC_MAT;
global MEASUREMENTS;
global VALID;
Measurements = MEASUREMENTS;
valid = VALID;

%% contd
nin = length(X);
featPts_mod = zeros(3,6);
for j=1:6
    featPts_mod(:,j) = X( (nin-3*(7-j)+1):(nin-3*(6-j)) );
end

RollAngles = zeros(1,length(nums));
for i=1:length(nums)
	RollAngles(i) = X(i)*pi/180;
end

%% get rotation from input angle
% given vector to be the x-axis
ExtMats = cell(1,length(nums));
RotMats = cell(1,length(nums));
for i=1:length(nums)
    vect = directions(:,i);
    x_axis = vect/norm(vect);
    roll_angle = RollAngles(i);
    
    % given theta as roll angle
    yhat = [0;0;1];
    zhat = cross(x_axis, yhat);
    y_axis = sin(roll_angle)*zhat + cos(roll_angle)*yhat;
    z_axis = cross(x_axis, y_axis);

    % put R together
    R = [x_axis y_axis z_axis];
    ExtMats{i} = [R translations(:,i)];
    RotMats{i} = R;
end

%% get predicted points
Predictions = cell(1,length(nums));

for i=1:length(nums)
    Ext = ExtMats{i};
    predFeatPts_imEquiv = K * Ext * [featPts_mod; ones(1,size(featPts_mod, 2))];
    predFeatPts_im = predFeatPts_imEquiv(1:2,:) ./ ([1;1]*predFeatPts_imEquiv(3,:));
    Predictions{i} = predFeatPts_im;
end

%% get errors per image
Errors = cell(1,length(nums));
for i=1:length(nums)
    errs = [];
    for j=1:6
        if valid{j}(i)
            e = norm(Predictions{i}(:,j) - Measurements{j}(:,i));
            errs = [errs e];
        end
    end
    Errors{i} = errs;
end

scores = cat(2, Errors{:});
%% sum errors per image
ImScores = zeros(1,length(nums));
for i=1:length(nums)
    sc = sum(Errors{i});
    ImScores(i) = sc;
end


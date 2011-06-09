%% read cam params from calibration results
%% calib*.mat is Calib_Results.mat, renamed.
function cams = readCamParamsFromCalibrationFunc()

linux = strcmp(getenv('OSTYPE'),'linux');
win32 = strcmp(getenv('OS'),'Windows_NT');
if win32
    directory = 'D:/Research/modelbasedtracking';
else
    directory = '/windows/D/Research/modelbasedtracking';
end

if ~exist('calibDir')
    calibDir = 'data/2010Jun18/calib'      
end

%% cam1
load([directory '/' calibDir '/' 'Calib_Results1.mat']);

%% get variables
ccErr = [1;0];                       % means our focal point is a bit off.
cams{1}.axis_angle = omc_ext;
cams{1}.trans = Tc_ext';
cams{1}.foc = fc;
cams{1}.principlePoint = cc+ccErr;
cams{1}.distortions = kc;
cams{1}.skew = alpha_c;
cams{1}.R = Rc_ext;
cams{1}.K = [fc(1) 0 cams{1}.principlePoint(1); 0 fc(2) cams{1}.principlePoint(2); 0 0 1];

%% cam2
load([directory '/' calibDir '/' 'Calib_Results2.mat']);

%% get variables
ccErr = [1;1];                       % means our focal point is a bit off.
cams{2}.axis_angle = omc_ext;
cams{2}.trans = Tc_ext;
cams{2}.foc = fc;
cams{2}.principlePoint = cc+ccErr;
cams{2}.distortions = kc;
cams{2}.skew = alpha_c;
cams{2}.R = Rc_ext;
cams{2}.K = [fc(1) 0 cams{2}.principlePoint(1); 0 fc(2) cams{2}.principlePoint(2); 0 0 1];

%% cam3
load([directory '/' calibDir '/' 'Calib_Results3.mat']);

%% get variables
ccErr = [1;1];                       % means our focal point is a bit off.
cams{3}.axis_angle = omc_ext;
cams{3}.trans = Tc_ext;
cams{3}.foc = fc;
cams{3}.principlePoint = cc+ccErr;
cams{3}.distortions = kc;
cams{3}.skew = alpha_c;
cams{3}.R = Rc_ext;
cams{3}.K = [fc(1) 0 cams{3}.principlePoint(1); 0 fc(2) cams{3}.principlePoint(2); 0 0 1];
%% get model data
close all; clear all; clc
model = 'LND_3'

readModelData
modelData.verts = verts;
modelData.edgeList = edgeList;
modelData.badinds = badinds;
modelData.name = model;

%% get image and calibration data
calibDir = 'data/2010Jun18/calib'                   % relative to modelbasedtracking home
readCamParamsFromCalibration(directory, calibDir)
nCam = length(cams);

%% process images
close all
imDir = 'data/2010Jun18';
pre = 'hand';
imNum = 2

processImages
for k=1:nCam
    cams{k}.canny = cannys{k};
    cams{k}.cannyInds = cannyInds{k};
    cams{k}.edgeNormals = edgeNormals{k};
end

%% initial guesses 
storedTransforms_2010Jun18
R_orig = R;
angs_orig = [r0*pi r1*pi r2*pi];
t_orig = t;
%% estimate transforms
[angles, trans] = estimateTransform(modelData,ims, cams, angs_orig, t_orig + [5 5 5]');
% [angles, trans] = estimateTransform(modelData,ims, cams, angles, t_orig, 6, 0);

showCorresp
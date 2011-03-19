function [array2d] = projectArray(X, camStruct)
% camStruct from readCamParamsFromCalibration[Func]

trans = camStruct.trans;
R = camStruct.R;
K = camStruct.K;

XCam = R*X;
XCam(1,:) = XCam(1,:) + trans(1);
XCam(2,:) = XCam(2,:) + trans(2);
XCam(3,:) = XCam(3,:) + trans(3);
XCamDivd(3,:) = ones(1,size(XCam, 2));
XCamDivd(1,:) = XCam(1,:) ./ XCam(3,:);
XCamDivd(2,:) = XCam(2,:) ./ XCam(3,:);
array2d = K*XCamDivd;
array2d = array2d(1:2,:);
% pixCoord = K*[XCam(1)/XCam(3); XCam(2)/XCam(3); 1];
% 
% r = pixCoord(2);
% c = pixCoord(1);
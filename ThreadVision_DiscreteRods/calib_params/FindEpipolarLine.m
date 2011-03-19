function [m,b] = FindEpipolarLine(paramsFrom,paramsTo,point, KFrom, RFrom, TFrom, KTo, RTo, TTo)
%finds the slope and y intercept of the epipolar line. We consider point to
%be on the from camera, and we are finding the epipolar line on the to
%camera.

%equation is of form row = m*col + b

point = [point(2);point(1)];

if nargin < 9
    [KFrom,RFrom,TFrom] = ParamsToMatrices(paramsFrom);
    [KTo,RTo,TTo] = ParamsToMatrices(paramsTo);
end


%%
%Find two world coordinates that the point projects to
%Fix ZCam=200 and ZCam=300
ZCam1 = 200;
ZCam2 = 300;

%ZCam1 = -RFrom(3,:)*ZWorld1+TFrom;
%ZCam2 = -RFrom(3,:)*ZWorld2+TFrom;

% XCam1 = inv(KFrom)*[point;1]*ZCam1;
% XCam2 = inv(KFrom)*[point;1]*ZCam2;

XCam1 = KFrom\[point;1]*ZCam1;
XCam2 = KFrom\[point;1]*ZCam2;


%XWorld1 = -inv(RFrom)*[XCam1(1:2);ZCam1] + TFrom;
%XWorld2 = -inv(RFrom)*[XCam2(1:2);ZCam2] + TFrom;
XWorld1 = -RFrom\([XCam1(1:2);ZCam1] + TFrom);
XWorld2 = -RFrom\([XCam2(1:2);ZCam2] + TFrom);

[r1,c1] = WorldCoordToPixel(XWorld1,paramsTo);
[r2,c2] = WorldCoordToPixel(XWorld2,paramsTo);

[m,b] = TwoPointsToLine([c1;r1],[c2;r2]);

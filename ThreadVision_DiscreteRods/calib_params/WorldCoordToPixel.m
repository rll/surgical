function[r,c] = WorldCoordToPixel(X,params, K, RWorldToCam, trans)

%params are given as: [focal length; principal points; translation;
%X is a 3dimensional vector, which is taken as (east,north,up)

if nargin < 4
    foc = params(1:2);
    princPoints = params(3:4);
    trans = params(5:7);
    rotVec = params(8:10);


    alpha = norm(rotVec);
    rotVec = rotVec / alpha;
    RMatForRod = [0, -rotVec(3), rotVec(2); rotVec(3), 0, -rotVec(1);  -rotVec(2), rotVec(1), 0];
    RWorldToCam = eye(3) + RMatForRod*sin(alpha) + RMatForRod^2*(1-cos(alpha));
    
    K = [foc(1), 0, princPoints(1); 0, foc(2), princPoints(2); 0, 0, 1];
end

%not sure why I need to rotate X by 180%, but it seems necessary...
%XCam = RWorldToCam*(-eye(3)*X+trans);
XCam = RWorldToCam*X+trans;



pixCoord = K*[XCam(1)/XCam(3); XCam(2)/XCam(3); 1];

r = pixCoord(2);
c = pixCoord(1);



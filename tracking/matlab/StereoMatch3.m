function P = StereoMatch3(cam1orCams,cam2orPts,cam3,x1,x2,x3)
if nargin == 2
    x1 = cam2orPts(:,1);
    x2 = cam2orPts(:,2);
    x3 = cam2orPts(:,3);
    cam2 = cam1orCams{2};
    cam3 = cam1orCams{3};
    cam1 = cam1orCams{1};
else
    cam1 = cam1orCams;
    cam2 = cam2orPts;
end

K1 = cam1.K;
R1 = cam1.R;
T1 = cam1.trans;
K2 = cam2.K;
R2 = cam2.R;
T2 = cam2.trans;
K3 = cam3.K;
R3 = cam3.R;
T3 = cam3.trans;

worldRay1 = -R1\(K1\[x1;1]);
worldRay2 = -R2\(K2\[x2;1]);
worldRay3 = -R3\(K3\[x3;1]);

A = zeros(9,6);
A(1:3, 1:3) = eye(3);
A(4:6, 1:3) = eye(3);
A(7:9, 1:3) = eye(3);
A(1:3,4) = worldRay1;
A(4:6,5) = worldRay2;
A(7:9,6) = worldRay3;

CamCoords = [-R1\T1; -R2\T2; -R3\T3];

P = A\CamCoords;
P = P(1:3);

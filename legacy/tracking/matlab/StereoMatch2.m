function P = StereoMatch2(cam1,cam2,x1,x2)
K1 = cam1.K;
R1 = cam1.R;
T1 = cam1.trans;
T1 = -R1\T1;
K2 = cam2.K;
R2 = cam2.R;
T2 = cam2.trans;
T2 = -R2\T2;

pp = cam1.principlePoint;
f = cam1.foc;
x = x1;
ray1 = [(x(1)-pp(1))/f(1); (x(2)-pp(2))/f(2);1];
worldRay1 = -R1\ray1;                                 % norm of ray is irrelevant
pp = cam2.principlePoint;
f = cam2.foc;
x = x2;
ray2 = [(x(1)-pp(1))/f(1); (x(2)-pp(2))/f(2);1];
worldRay2 = -R2\ray2;

A = zeros(6,5);
A(1,1) = 1;
A(2,2) = 1;
A(3,3) = 1;
A(4,1) = 1;
A(5,2) = 1;
A(6,3) = 1;
A(1:3,4) = worldRay1;
A(4:6,5) = worldRay2;

CamCoords = [T1;T2];

P = A\CamCoords;
P = P(1:3);

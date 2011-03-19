function [K,R,T] = ParamsToMatrices(params)

foc = params(1:2);
princPoints = params(3:4);
T = params(5:7);
rotVec = params(8:10);


alpha = norm(rotVec);
rotVec = rotVec / alpha;
RMatForRod = [0, -rotVec(3), rotVec(2); rotVec(3), 0, -rotVec(1);  -rotVec(2), rotVec(1), 0];
R = eye(3) + RMatForRod*sin(alpha) + RMatForRod^2*(1-cos(alpha));


K = [foc(1), 0, princPoints(1); 0, foc(2), princPoints(2); 0, 0, 1];

end
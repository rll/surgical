function XNew = affineTransform(R, t, X)

% wherein X is a 3xn list of 3d points
XNew = R*X;
XNew(1,:) = XNew(1,:) + t(1);
XNew(2,:) = XNew(2,:) + t(2);
XNew(3,:) = XNew(3,:) + t(3);

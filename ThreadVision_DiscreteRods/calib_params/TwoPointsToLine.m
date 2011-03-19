function [m,b] = TwoPointsToLine(X1,X2)

m = (X1(2)-X2(2))/(X1(1)-X2(1));

b = X1(2)-m*X1(1);
function C = circDiff(A, B, lim) 

%%
A = mod(A, lim);
B = mod(B, lim);
C1 = abs(A-B);
C2 = lim - C1;
bool = abs(C1) < abs(C2);
C = C1.*bool + C2.*(~bool);


%% P = list of peaks
%% T = theta list 
%% rhos = rho list 
hold on
xx = 1:nx;
for i=1:size(P,1)
    peak = P(i,:);
    theta = T(peak(2)) * pi / 180;
    m = -cot(theta);
    r = rhos(peak(1));
    b = r/sin(theta);
    yy = m*xx + b;
    plot(xx,yy,'LineWidth',2)
end
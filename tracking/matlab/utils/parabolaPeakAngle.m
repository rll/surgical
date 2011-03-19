function th = parabolaPeakAngle(thetas, yy, mode)
% yy(2) is max(yy)

if nargin < 3 || strcmp(mode, 'rad')
    lim = pi;
    mode = 'rad';
else
    lim = 180;
end

if length(thetas) ~= 3 || length(yy) ~= 3
    error('need 3 points for quadratic interpolation');
end

for j=[1,3]
    if thetas(j) - thetas(2) > lim
        thetas(j) = thetas(j) - 2*lim;
    elseif thetas(2) - thetas(j) > lim
        thetas(j) = thetas(j) + 2*lim;
    end

    if thetas(j) - thetas(2) > lim/2
        thetas(j) = thetas(j) - lim;
    elseif thetas(2) - thetas(j) > lim/2;
        thetas(j) = thetas(j) + lim;
    end
end

% thetas
th = parabolaPeak(thetas, yy);
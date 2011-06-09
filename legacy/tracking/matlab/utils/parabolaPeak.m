function x = parabolaPeak(xx, yy)


if length(xx) ~= 3
    error('parabolaPeak: please supply exactly 3 x points');
end
if length(yy) ~= 3
    error('parabolaPeak: please supply exactly 3 y points');    
end

if size(xx,1) ~= 1
    xx = xx';
end
if size(yy,2) ~= 1
    yy = yy';
end

A = vander(xx);
abc = A\yy;

x = -abc(2)/(2*abc(1));


end
function scatter2(arr, varargin)

sz = size(arr);
if sz(1) == 2
    xx = arr(1,:);
    yy = arr(2,:);
elseif sz(2) == 2
    xx = arr(:,1);
    yy = arr(:,2);
end

l = length(varargin);
options = cell(1,l);
for i=1:l
    opt = varargin(i);
    options{i} = opt{1};
end

if l == 0
    scatter(xx,yy, '.')
elseif l == 1
    scatter(xx,yy,options{1});
elseif l == 2
    scatter(xx,yy,options{1}, options{2});
elseif l == 3
    scatter(xx,yy,options{1}, options{2}, options{3});
elseif l == 4
    scatter(xx,yy,options{1}, options{2}, options{3}, options{4});
elseif l == 5
    scatter(xx,yy,options{1}, options{2}, options{3}, options{4}, options{5});
else
    scatter(xx,yy)
end
    
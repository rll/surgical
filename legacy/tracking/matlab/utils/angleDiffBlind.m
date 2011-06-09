function diffs = angleDiffBlind(a1, a2, mode) 
%% Direction-blind angle difference; treats theta and 180+theta as the same angle. Returns the smallest
%% difference possible. Returns absolute values.
%%
%% a1 must be a scalar, but a2 may be an array. 
%% diffs will be of size [1,numel(a2)]

if nargin < 3 || strcmp(mode, 'rad')
    lim = pi;
    mode = 'rad';
else
    lim = 180;
    mode = 'deg';
end

d1s = angleDiff(a1, a2, mode);
d2s = angleDiff(a1, mod(a2, 2*lim) + lim, mode);

diffs = min(d1s, d2s);
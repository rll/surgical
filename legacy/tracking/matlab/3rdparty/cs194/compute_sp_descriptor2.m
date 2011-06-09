% Compute spatial pyramid histogram descriptor.
%
% Inputs:
%
%   mag        - an M x N matrix containing the magnitude of the gradient
%                at every pixel
%
%   obin       - an M x N matrix in which each entry is an integer in the 
%                range [1,n_ori_bins] indicating the quantized orientation
%                channel to which the pixel belongs
% 
%   n_levels   - the number of levels of the spatial pyramid
%
%   n_ori_bins - the number of orientations used when quantizing
%                gradient features
%
% Output:
%
%   descriptor - a column vector containing the descriptor for the image
%
% Note that the length of the descriptor may vary depending on your
% implementation, but you should produce the same length descriptor
% for each image.

function descriptor = compute_sp_descriptor2(magMap, obinMap, n_levels, n_ori_bins,  thresh)
[height, width] = size(magMap);
descriptor = [];
for l=0:(n_levels-1)
    n = 2^(l);
    sy = height/n;
    sx = width/n;
    if l == 0       % to make it consistent with the old implementation
        weight = 1;
    else
        weight = 2^(l-1);
    end
    weight
%     weight = 2^l;

    levelDescs = zeros(n_ori_bins, n, n);
    for iy=0:(n-1)
        ay = floor(iy*sy)+1;
        by = floor(iy*sy + sy);
        for ix=0:(n-1)
            ax = floor(ix*sx)+1;
            bx = floor(ix*sx + sx);
%             [ay, by, ax, bx]
            magCell = magMap(ay:by, ax:bx);
            obinCell = obinMap(ay:by, ax:bx);
            cellHist = compute_histogram2(obinCell, magCell, n_ori_bins, thresh);
            levelDescs(:, ix+1, iy+1) = cellHist * weight;
        end
    end
    
    descriptor = [descriptor; levelDescs(:)];
end

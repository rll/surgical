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

function descriptor = compute_sp_descriptor(mag, obin, n_levels, n_ori_bins)

%First level
desc = compute_histogram2(obin, mag, n_ori_bins);
descriptor = build_descriptor(obin, mag, n_levels, n_ori_bins, desc, 0)';

% this dummy implementation returns a fixed-length random vector
%descriptor = rand([500 1]);
end

function descr = build_descriptor(obin, mag, n_levels, n_ori_bins, desc, weight)
    if (n_levels == 1)
        descr = desc;
        return
    end
    lx = floor(size(obin,2)/2);
    ly = floor(size(obin,1)/2);
%     [ly, lx]
    
    o1 = obin(1:floor(size(obin,1)/2), 1:floor(size(obin,2)/2));
    o2 = obin(1:floor(size(obin,1)/2), floor(size(obin,2)/2)+1:end);
    o3 = obin(floor(size(obin,1)/2)+1:end, 1:floor(size(obin,2)/2));
    o4 = obin(floor(size(obin,1)/2)+1:end, floor(size(obin,2)/2)+1:end);
    m1 = mag(1:floor(size(obin,1)/2), 1:floor(size(obin,2)/2));
    m2 = mag(1:floor(size(obin,1)/2), floor(size(obin,2)/2)+1:end);
    m3 = mag(floor(size(obin,1)/2)+1:end, 1:floor(size(obin,2)/2));
    m4 = mag(floor(size(obin,1)/2)+1:end, floor(size(obin,2)/2)+1:end);
    w = 2^weight;
    descr = [desc compute_histogram2(o1, m1, n_ori_bins)*w ...
        compute_histogram2(o2, m2, n_ori_bins)*w ...
        compute_histogram2(o3, m3, n_ori_bins)*w ...
        compute_histogram2(o4, m4, n_ori_bins)*w];
    descr = build_descriptor(o1, m1, n_levels-1, n_ori_bins, descr, weight+1); 
    descr = build_descriptor(o2, m2, n_levels-1, n_ori_bins, descr, weight+1); 
    descr = build_descriptor(o3, m3, n_levels-1, n_ori_bins, descr, weight+1); 
    descr = build_descriptor(o4, m4, n_levels-1, n_ori_bins, descr, weight+1); 
end

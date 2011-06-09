% Compute spatial pyramid (sp) histogram of gradients (hog) descriptor for an
% image.
%
% Inputs:
%
%   I          - the image
%
%   n_ori_bins - the number of orientations to use when quantizing
%                gradient features in the descriptor computation
%
%   n_levels   - the number of levels of the spatial pyramid
%
% Output:
%
%   descriptor - a column vector containing the descriptor for the image
%
% Note that the length of the descriptor may vary depending on your
% implementation, but you should produce the same length descriptor
% for each image.

function descriptor = compute_sp_hog(im, nbins, nlevels, thresh)

%% compute gradient magnitude and orientation bin assignment at each pixel
[mag, obin] = compute_ori_gradients2(im, nbins);
mag;

% create spatial pyramid histogram descriptor
descriptor = compute_sp_descriptor2(mag, obin, nlevels, nbins, thresh);

% Compute gradient magnitude and orientation bin assignment.
%
% Inputs:
%
%   I          - the image in the form of an M x N matrix (grayscale) or
%                an M x N x 3 matrix (color)
%
%   nbins - the number of orientations to use when quantizing
%                gradient features in the descriptor computation
%
% Outputs:
%
%   mag        - an M x N matrix containing the magnitude of the gradient
%                at every pixel
%
%   obin       - an M x N matrix in which each entry is an integer in the 
%                range [1,nbins] indicating the quantized orientation
%                channel to which the pixel belongs
function [mag, obin] = compute_ori_gradients2(im, nbins)
%% body
sig1 = 2;
sig2 = 2;

% convert input to floating-point if its currently stored as integer
if ~isfloat(im)
    im = im2double(im);
end

% convert a color image into a grayscale image
if ndims(im) == 3,
    im = 0.3*im(:,:,1) + 0.59*im(:,:,2) +  0.11*im(:,:,3);
end

dG1 = GaussianDerivative(sig1);
dG2 = GaussianDerivative(sig2)';
Cx = imfilter(im, dG1, 'replicate', 'conv', 'same');
Cy = imfilter(im, dG2, 'replicate', 'conv', 'same');
mag = sqrt(Cx.*Cx + Cy.*Cy);

Oris = atan2(Cy, Cx);
obin = ceil((Oris+pi) * (1/(2*pi)) * nbins);
obin(obin == 0) = 1;
%     figure; imagesc(obin);
%     waitforbuttonpress


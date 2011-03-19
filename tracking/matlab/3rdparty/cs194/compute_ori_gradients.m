% Compute gradient magnitude and orientation bin assignment.
%
% Inputs:
%
%   I          - the image in the form of an M x N matrix (grayscale) or
%                an M x N x 3 matrix (color)
%
%   n_ori_bins - the number of orientations to use when quantizing
%                gradient features in the descriptor computation
%
% Outputs:
%
%   mag        - an M x N matrix containing the magnitude of the gradient
%                at every pixel
%
%   obin       - an M x N matrix in which each entry is an integer in the 
%                range [1,n_ori_bins] indicating the quantized orientation
%                channel to which the pixel belongs
function [mag, obin] = compute_ori_gradients(I, n_ori_bins)
%% body
step = 0;
sig1 = 2;
sig2 = 2;

% convert input to floating-point if its currently stored as integer
if ~isfloat(I)
    I = im2double(I);
end

% onvert a color image into a grayscale image
if ndims(I) == 3,
    I = 0.3*I(:,:,1) + 0.59*I(:,:,2) +  0.11*I(:,:,3);
end

if ~step
    %%Convolution with Gaussian
    x = floor(-3*sig1):ceil(3*sig1);
    G1 = gaussmf(x,[sig1 0]) / (sig1 * sqrt(2*pi));

    x1 = floor(-3*sig2):ceil(3*sig2);
    G2 = gaussmf(x1,[sig2 0]) / (sig2 * sqrt(2*pi));
    C = conv2(im2double(I), G1'*G2, 'same');
%     figure; imshow(C/max(C(:))); colormap gray
    [Cx, Cy] = gradient(C);

    Cx2 = Cx.*Cx;
    Cy2 = Cy.*Cy;
    Cx2y2 = Cx2 + Cy2;
    mag = sqrt(Cx2y2);

%     Cyx = Cy ./ (Cx + .000001); %Adding epsilon prevents division by zero
%     O = atan(Cyx);
%     obin = ceil(((O + pi/2 * ones(size(O)))/pi) * n_ori_bins);
    Oris = atan2(Cy, Cx);
    obin = ceil((Oris+pi) * (1/(2*pi)) * n_ori_bins);
%     figure; imagesc(obin);
%     waitforbuttonpress
else
    %%Convolution with Step Function
    for (i=1:8)
    S = [-1 -1 -1;0 0 0 ;1 1 1];
    
    Cy = conv2(I, S, 'same');
    Cx = conv2(I, S', 'same');

    Cx2 = Cx.*Cx;
    Cy2 = Cy.*Cy;
    Cx2y2 = Cx2 + Cy2;
    mag = sqrt(Cx2y2);

    Cyx = Cy ./ (Cx + .000001);
    O = atan(Cyx);
    obin = ceil(((O + pi/2 * ones(size(O)))/pi) * n_ori_bins);
    end
end

end


function [ magnitude, orientations] =sobel (I,n_ori_bins)
    S = [-1 -1 -1;0 0 0 ;1 1 1];
    
    Cy = conv2(I, S, 'same');
    Cx = conv2(I, S', 'same');

    Cx2 = Cx.*Cx;
    Cy2 = Cy.*Cy;
    Cx2y2 = Cx2 + Cy2;
    magnitude = sqrt(Cx2y2);

    Cyx = Cy ./ (Cx + .000001);
    O = atan(Cyx);
    orientations = ceil(((O + pi/2 * ones(size(O)))/pi) * n_ori_bins);
end
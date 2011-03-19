% View oriented gradients for a category.
% This function may help you debug your oriented gradient computation.
function view_ori_gradients(categ, imgset)

% default arguments
if (nargin < 1)
   categ = 'airplanes';
end

if (nargin < 2)
   imgset = 'train';
end

% get list of images for the category
dd = dir(['data/' imgset '/' categ '/*.jpg']);

% loop over images, compute and display gradients
for id = 1:length(dd),
    I = imread(['data/' imgset '/' categ '/' dd(id).name]);
    
    [mag,obin] = compute_ori_gradients(I,8);
   
    figure(1); imshow(I); axis image; title('image');

    figure(2);
    subplot(1,2,1); imagesc(mag);  axis image; title('gradient magnitude');
    subplot(1,2,2); imagesc(obin); axis image; title('orientation bin');
    pause; % press any key to continue to next image
end;

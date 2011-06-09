%% This was written before I discovered you could undistort images in
%% color. It "unwrinkles" the background. 
%% It calculates 1) a foreground mask and 2) an "unwrinkled" grayscale
%% image by setting all background pixels to the same color. 

%% If I had known how to undistort color images, I probably would have
%% save an unwrinkled color image instead.

%% For every image, mask2 is the final mask. 

directory = RemoteHomeDirectory;
imDir = 'data/2010Jul23/rect_color/';
outDir = 'data/2010Jul23/gray/';
pre = 'knottie';
suf = '.tif';
opt = 'initialmagnification'
greenhue = .42;
greenval = .2471;

nums = 400:499;
for k=1:3
    for i=1:length(nums)
        imNum = nums(i);
        %%
        filepath = [directory imDir pre int2str(k) '-_rect_color' int2str(imNum) suf];
        im = imread(filepath);
        
        hsv = rgb2hsv(im2double(im));
        hue = hsv(:,:,1);
        sat = hsv(:,:,2);
        val = hsv(:,:,3);
        img = rgb2gray(im);
        
        mask_sat = sat < .5;
        mask2 = bwmorph(mask_sat, 'open');  % noise reduction 
        
        img3 = img;
        img3(~mask2) = 55;
        
        if randi(100, 1) == 1
            figure; imshow(img3, opt ,50)
        end
        %%
        outfile = [directory outDir pre int2str(k) '-' num2str(imNum) '.tif']
        imwrite(img3, outfile);
    end
end

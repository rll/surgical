for i=1:length(ns)-1
    imNum1 = ns(i);
    imNum2 = ns(i+1);
    i1 = find(nums == imNum1);
    i2 = find(nums == imNum2) ;  
    l = imNum2 - imNum1 + 1;
    d = angs(i+1) - angs(i);
    dang = d/(l-1);
    
    aa = angs(i):dang:angs(i+1);
    if d == 0
        aa = angs(i)*ones(1,l);
    end
    if d > 180
        d = angs(i+1) - (angs(i) + 360);
        dang = d/(l-1);
        aa = (angs(i) + 360):dang:angs(i+1);
    end
    
    aa
    init_angs(i1:i2) = aa;
end

%% valid -> valid2
valid2 = cell(1,length(nums));

for i=1:length(nums)
    vv = zeros(1,6);
    for j=1:6
        vv(j) = valid{j}(i);
    end
    valid2{i} = vv == 1;
end


%% don't plot
close all
for i=51:60
    imNum = nums(i);
    im = imread([directory '/' imDir '/' pre int2str(k) '-_rect' int2str(imNum) '.tif']);
    
    figure; imshow(im*3); hold on
    pp = Predictions{i};
    scatter(pp(1,:), pp(2,:));
    
    for j=1:6
        mm = Measurements{j}(:, i);
        scatter(mm(1,:), mm(2,:), 'g');
    end
end


%% grayscale images for rectification
directory = RemoteHomeDirectory;
imDir = 'data/2010Jul23/';
pre = 'knottie';
suf = '.tiff';
opt = 'initialmagnification'
greenhue = .42;

nums = 573;
for k=1:3
    for i=1:length(nums)
        imNum = nums(i);
        %%
        filepath = [directory imDir pre int2str(k) '-' int2str(imNum) suf]
        im = imread(filepath);
        
        hsv = rgb2hsv(im);
        hue = hsv(:,:,1);
        sat = hsv(:,:,2);
        huediff = circDiff(hue, .42, 1);
        
        figure; imshow(hue, opt, 50);
        figure; imshow(huediff, opt, 50);
        figure; imshow(sat, opt, 50);
        figure; imshow(huediff./sat, opt, 50);
        
        img = rgb2gray(im);
    end
end

%% command history
figure; imshow(sat);
help bwmorph
figure; imagesc(sat < .5)
mask = sat<.5;
mask2 = bwmorph(sat, 'close');
figure; imagesc(mask2)
max(mask2(:))
min(mask2(:))
figure; imagesc(bwmorph(sat, 'dilate'))
mask2 = bwmorph(mask, 'close');
figure; imagesc(mask2)
figure; imagesc(mask)
figure; imagesc(bwmorph(mask, 'dilate'))
figure; imagesc(bwmorph(mask, 'erod'))
figure; imagesc(bwmorph(mask, 'erode'))
figure; imagesc(bwmorph(~mask, 'close'))
figure; imagesc(~bwmorph(~mask, 'close'))
mask3 = ~bwmorph(~mask, 'close');
diff = abs(mask - mask3);
figure; imagesc(diff)
mask4 = bwmorph(mask, 'open');
figure; imagesc(mask4)
figure; imagesc(abs(mask4-mask3));
max(abs(mask4(:), mask3(:)))
max(abs(mask4(:)- mask3(:)))
[ii,jj] = find(mask4 ~= mask3)
figure; imagesc(mask4 == mask3);
close all
mask = mask3;
imgm = img .* mask;
imgm = im2double(img ) .* im2double(mask);
figure; imshow(imgm, opt, 50)
imgm = im2double(im) .* im2double(mask);
figure; imshow(imgm, opt, 50)
figure; imshow(edge(imgm, 'canny', [.2,.3], 2))
figure; imshow(edge(imgm, 'canny', [..05,.3], 2), opt, 50)
figure; imshow(edge(imgm, 'canny', [.05,.3], 2), opt, 50)
figure; imshow(edge(imgm, 'canny', [.1,.2], 2), opt, 50)
figure; imshow(edge(imgm, 'canny', [.05,.2], 2), opt, 50)
figure; imshow(edge(imgm, 'canny', [.05,.1], 2), opt, 50)
get(gca, 'currentpoint')
img(386,111)
figure; imshow(img, opt, 50)
figure; imshow(im, opt, 50)
get(gca, 'currentpoint')
im(595,295)
imgm2 = im;
imgm2(mask) = 92;
figure; imshow(imgm2, opt, 50)
imgm2 = im;
imgm2(~mask) = 92;
figure; imshow(imgm2, opt, 50)
directory = RemoteHomeDirectory;
imDir = 'data/2010Jul23/';
pre = 'knottie';
suf = '.tiff';
greenhue = .42;
filepath = [directory imDir pre int2str(k) '-' int2str(imNum) suf]
im = imread(filepath);
hsv = rgb2hsv(im);
hue = hsv(:,:,1);
sat = hsv(:,:,2);
huediff = circDiff(hue, .42, 1);
img = rgb2gray(im);
figure; imagesc(sat <.5);
img2 = img;
figure; imshow(im, opt, 50)
get(gca, 'currentpoint')
img(495, 407)
figure; imshow(img, opt, 50)
maks = sat < .5;
mask = sat < .5;
img2(mask) = 52;
figure; imshow(img2, opt, 50)
img2 = img;
img2(~mask) = 52;
figure; imshow(img2, opt, 50)
mask2 = bwmorph(mask, 'open');
img3 = img;
img3(~mask2) = 52;
figure; imshow(img3, opt ,50)
im2 = im;
g = im2(:,:,2);
im2(:,:,2) = g(~mask2)*3;
g(~mask2) = g(~mask2)*3;
im2(:,:,2) = g(~mask2)*3;
im2(:,:,2) = g;
figure; imshow(im2, opt, 50);
close 1
close 2
close 3
close 4
close 5
figure; imshow(img2, opt, 50)
outpath
imwrite(img2, outpath, 'tif')

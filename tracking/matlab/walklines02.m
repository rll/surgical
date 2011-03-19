% dbstop if error
set(0,'DefaultFigureWindowStyle','normal');
opt = 'initialmagnification'

directory = RemoteHomeDirectory;
imDir = 'data/2010Sept27/rect';
pre = 'test';

nx = 1280;
ny = 960;
xlimls = [-1 -1 -1];
xlimrs = [985 nx+1 1031];
ylimts = [212 220 -1];

close all
searchSide = 150;
maskSide = 350;
cropSide = 250;
side = 'left';

nums = [36:71 84:92 305:337 406:492 569:579];
startpts = [ [824;281] [681;217] [475;187] [483;220] [523;203]];
starti = [36 84 305 406 569];

ex_loc = [806;509];        
ex_rho = -185;
ex_angle = -45;

%% hello world.
for k=1:1
    %% iter
    
    nums2 = 406:492
    locs = zeros(2, length(nums2));
    lineThetas = cell(1,length(nums2));
    lineRhos = cell(1,length(nums2));
    
    for i2=1:length(nums2);
        %% iter
        imNum = nums2(i2);
        if ismember(imNum, starti)
            ix = find(starti == imNum);
            ex_loc = startpts(:,ix)
        end

        locateMid
        midpt
        lineThetas{i2} = thetaPeaks;
        lineRhos{i2} = rhoPeaks;
%         figure; imshow(img); hold on
%         scatter(midpt(1), midpt(2));
        
        x = round(midpt(1)); y = round(midpt(2));
        
        dispim = reshape([img img img], [ny, nx, 3]);
        dispim(y, x,3) = 255;
        dispim(y, x+1,3) = 255;
        dispim(y, x-1,3) = 255;
        dispim(y+1, x,3) = 255;
        dispim(y-1, x,3) = 255;
        dispim(y, x,2) = 0;
        dispim(y, x+1,2) = 0;
        dispim(y, x-1,2) = 0;
        dispim(y+1, x,2) = 0;
        dispim(y-1, x,2) = 0;

        x0 = max(1, x - 150);
        x1 = min(nx, x + 150);
        y0 = max(1, y - 150);
        y1 = min(ny, y + 150);
        cropped = dispim(y0:y1, x0:x1,:);

        doplot2 = false;
        if doplot2
            figure; imshow(cropped);
        end
        
%         imwrite(cropped, ...
%             [RemoteHomeDirectory 'results/band/band' num2str(k) ...
%             '-' num2str(imNum) '.tiff']);

        locs(:, i2) = midpt;
        ex_loc = midpt;
        ex_angle = avgAngle; 
        ex_rho = avgRho;
    end
end

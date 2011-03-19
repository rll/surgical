%% Use to quickly label points on images. 
%% May need to change variables directory, nums, visible_nums, and pre. 
%% The array of points end up in pp. 

%% When it first starts up, it asks you to zoom to your region of interest.
%% When you're done, press a letter key. 

%% Then click on the point. After the click, it automatically goes to the
%% next image, at the same zoom level. After 10 clicks, you will have to
%% re-zoom. 

%% If at any time you need to re-zoom, press 'z' before you click, and
%% press any letter key to get out of zoom mode. You only get one chance at
%% zoom mode per image, though. Once you get out, you have to click on a
%% point. 

%% If the point in question is not visible on the current image, press 'q'
%% and zeros will be recorded in pp. 

close all; clc

nums = 15:30:870;
% visible_nums = [406:413 460:476]; % pin1
% visible_nums = [423:452 485:492]; % pin2
% visible_nums = [406:440 468:492]; % Corner A1
% visible_nums = [406:447 469 478:492]; % Corner A2
% visible_nums = [446 456:469]; % Corner B1
% visible_nums = [445:465]; % Corner B2
visible_nums = nums;    % top and bottom

bool = ismember(nums, visible_nums);
ii = find(bool);

locs = zeros(2, length(nums));
pre = 'jinna';
k=1;
nx = 1280;
ny = 960;

fig = figure;
pp = zeros(2, length(nums));

for k=1:3
    
    for j=1:length(ii);
        c = 'a';
        i = ii(j);
        imNum = nums(i);
        directory = RemoteHomeDirectory;
        imDir = 'data/2010Aug18/closed';
        filename = [pre int2str(k) '-_rect_color' int2str(imNum) '.tif']
        im = imread([directory '/' imDir '/' filename]);

        imbright = rgb2gray(im2double(im))*2;

        if ~(mod(j,10) == 1)
            figure(fig); hold on
        elseif j > 1
            close(fig);                 % after 10 draws on the same image, it gets pretty slow. 
            fig = figure;
        end
        
        if mod(j,10) == 1
            imshow(imbright, 'initialmagnification', 50); 
        else 
            imshow(imbright);
        end
        
        if mod(j,10) == 1        
            disp('zooming')
            zoom on
            waitfor(fig, 'currentcharacter');
            zoom off;
        end

        disp('waiting for click or press...');
        button = waitforbuttonpress;

        if button == 1
            c = get(gcf, 'currentcharacter'); 
        end

        if c == 'q'
            disp('no visible point');
            c = 'a';
        else
            if c == 'z'
                zoom on
                waitfor(fig, 'currentcharacter');
                zoom off;
                c = 'a';
                disp('waiting for button press...');
                waitforbuttonpress;
            end
            p = get(gca, 'currentpoint');
            p = p(1, 1:2)';
            pp(:,i) = p;
        end
    end
end



%% don't display
figure;
for j=1:length(nums)
    imNum = nums(j);
    filename = [pre int2str(k) '-_rect' int2str(imNum) '.tif']
    im = imread([directory '/' imDir '/' filename]);
    imshow(im*4); hold on
    scatter(pp(1,ii(j)), pp(2,ii(j)), 'm');
    waitforbuttonpress
    hold off
end
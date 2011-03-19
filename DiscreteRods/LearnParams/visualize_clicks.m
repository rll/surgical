load ../../vision/captures/ribbon_dots_points.txt

NUM_POINTS = 68;
data = ribbon_dots_points;
IM_PATH1 = '~/rll/code/trunk/surgical/vision/captures/ribbon_dots1-';
IM_PATH2 = '~/rll/code/trunk/surgical/vision/captures/ribbon_dots2-';
IM_PATH3 = '~/rll/code/trunk/surgical/vision/captures/ribbon_dots3-';
% process data, and setup try_thread_param_noniso, projected:
% view_process_points
% errors 16, 26, 56^, 60*
% smooth: 6, 8, 10, 17, 18, 19, 20, 21, 27, 29, 30, 31, 38, 40, 43, 46, 48,
% 49, 52, 54, 57, 58
% shift: 34, 35, 36, 39, 45, 50
for i = 64:64
    figure;
    subplot(2,3,[1 3]);
    frame = data(i,2:end);
    frame_pts = zeros(NUM_POINTS, 3);
    even_pts = zeros(NUM_POINTS/2, 3);
    odd_pts = zeros(NUM_POINTS/2, 3);
    hold on;
    pair = zeros(2, 3);
    for j = 1:NUM_POINTS
        frame_pts(j,:) = [frame(1, 3*j - 2), frame(1,3*j - 1), frame(1,3*j)]; 
        text(frame_pts(j,1), frame_pts(j,2),frame_pts(j,3), num2str(j));
        if (mod(j,2) == 0)
            even_pts(int32(j/2), :) = frame_pts(j,:);
            pair(2,:) = frame_pts(j,:);
            plot3(pair(:,1), pair(:,2), pair(:,3));
        else
            odd_pts(int32(j/2), :) = frame_pts(j,:);
            pair(1,:) = frame_pts(j,:);
        end
    end
    hold on;
    even_pts
    odd_pts
    [smooth(odd_pts(:,1)) smooth(odd_pts(:,2)), smooth(odd_pts(:,3))]

    plot3(even_pts(:,1), even_pts(:,2), even_pts(:,3));
    plot3(odd_pts(:,1), odd_pts(:,2), odd_pts(:,3));
    plot3((even_pts(:,1)+odd_pts(:,1))/2, (even_pts(:,2)+odd_pts(:,2))/2, (even_pts(:,3)+odd_pts(:,3))/2, 'color', 'r');
    plot3(smooth(even_pts(:,1)), smooth(even_pts(:,2)), smooth(even_pts(:,3)), 'color', 'g');
    plot3(smooth(odd_pts(:,1)), smooth(odd_pts(:,2)), smooth(odd_pts(:,3)),'color', 'g');
    view(3);
    title(i);
    axis equal tight;
%     subplot(2,3,4);
%     img = imread([IM_PATH1, num2str(i), '.tif']);
%     imshow(img);
%     subplot(2,3,5);
%     img = imread([IM_PATH2, num2str(i), '.tif']);
%     imshow(img); 
%     subplot(2,3,6);
%     img = imread([IM_PATH3, num2str(i), '.tif']);
%     imshow(img);
    set(gcf,'Position',[300 20 800 800])


%    input(num2str(i));
%    close all;
end

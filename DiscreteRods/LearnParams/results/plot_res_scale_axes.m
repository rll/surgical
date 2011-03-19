files = {'ec2_1_28_log/suturenylon_combined.txt',
'ec2_1_28_log/sutureblack_combined.txt',
'ec2_1_28_log/suturepurple_combined.txt',};
titles = {'Nylon',
          'Silk',
          'Vicryl'};
x_label = 'log(\alpha))';
y_label = 'log(\beta / \alpha)';
z_label = 'Mean Error (mm^2)';

sorted = {};
sorted_median = {};

for file_ind=1:length(files)
    all_data = load(files{file_ind});
    all_data(:,4) = mean(all_data(:,15:end).^2,2);
    to_plot = all_data;
    to_plot(:,2) = log2(to_plot(:,2)./to_plot(:,1));%./2.^to_plot(:,1));    
    to_plot(:,1) = log2(to_plot(:,1));
    to_plot = to_plot(find(to_plot(:,1) >= 0.4),:);
    %to_plot(:,2) = log2(to_plot(:,2));
    
    if (strcmp(titles{file_ind}, 'Silk'))
      to_plot = to_plot(find(to_plot(:,2) <= 1.51),:);
    %else
    %  to_plot = to_plot(find(to_plot(:,2) <= 2.51),:);
    end

    %{
    figure;
    scatter3(to_plot(:,1), to_plot(:,2), to_plot(:,4),'filled');
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title(titles{file_ind});
    %}

    
    %heat plot
    h = figure;
    [x,y] = meshgrid(to_plot(:,1), to_plot(:,2));
    x = unique(x);
    y = unique(round(y*1e3)/1e3);
    Z = zeros(length(y), length(x));
    
    for i=1:size(to_plot,1)
        Z(find(abs(y - to_plot(i,2))<1e-3), find(abs(x - to_plot(i,1)) < 1e-3)) = to_plot(i,4);
    end
    surf(x,y,Z);
    xlabel(x_label,'FontSize',14);
    ylabel(y_label,'FontSize',14);
    zlabel(z_label,'FontSize',14);
    %title(titles{file_ind}, 'FontSize',12);
    view([80, 18]);
    print(h, '-djpeg', ['~/papers/trunk/2011_ICRA_JavdaniTandonTangGoldbergAbbeel-ThreadModeling/learning_imgs/' titles{file_ind} '.jpeg'], '-r125');

    
    
    sorted{file_ind} = sortrows(all_data,4);
    sorted{file_ind}(:,3) = sorted{file_ind}(:,2)./sorted{file_ind}(:,1);
    sorted{file_ind}(:,1) = 0;
    sorted{file_ind}(:,2) = 0;



%{
    %plot everything for median
    for_median = to_plot;
    for row_ind=1:size(for_median,1)
        for_median(row_ind,4) = median(to_plot(row_ind,5:end));
    end

    %
    figure;
    scatter3(for_median(:,1), for_median(:,2), for_median(:,4), 'filled');
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title([titles{file_ind} ' median']);
    %
    
    %heat plot
    figure;
    for i=1:size(to_plot,1)
        Z(find(abs(y - for_median(i,2))<1e-3), find(abs(x - for_median(i,1)) < 1e-3)) = for_median(i,4);
    end
    surf(x,y,Z);
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title([titles{file_ind} ' median']);

    
    
    sorted{file_ind} = sortrows(to_plot,4);
   





    sorted_median{file_ind} = sortrows(for_median,4);
    
    %}
end
% 
% for file_ind=1:length(files)
%     all_data = load(files{file_ind});
%     to_plot = all_data(:,1:4);
%     to_plot(:,2) = log2(to_plot(:,2)./to_plot(:,1));    
%     %to_plot = to_plot(find(to_plot(:,2) < 2),:);
%     figure;
%     scatter3(to_plot(:,1), to_plot(:,2), to_plot(:,4));
%     xlabel(x_label);
%     ylabel(y_label);
%     zlabel(z_label);
%     title(titles{file_ind});
% end
    

    

% 
% figure;
% A(:,4) = log(A(:,4));
% [x,y] = meshgrid(A(:,1), A(:,2));
% x = unique(x);
% y = unique(y);
% Z = zeros(length(y), length(x));
% 
% 
% for i=1:size(A,1)
%     Z(find(y==A(i,2)), find(x==A(i,1))) = A(i,4);
% end
% surf(x,y,Z);
% xlabel('bend');
% ylabel('twist');
% zlabel('score');
% title('purple');
% figure;
% scatter3(A(:,1), A(:,2), A(:,4));
% xlabel('bend');
% ylabel('twist');
% zlabel('score');
% title('purple');


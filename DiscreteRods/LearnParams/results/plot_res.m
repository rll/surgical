%files = {'ec2_1_27_set3/suturenylon_combined',
%'ec2_1_27_set3/sutureblack_combined',
%'ec2_1_27_set3/suturepurple_combined'};
 
files = {'ec2_1_28_iso/suturenylon_combined.txt',
'ec2_1_28_iso/sutureblack_combined.txt',
'ec2_1_28_iso/suturepurple_combined.txt'};
 

titles = {'nylon',
          'black',
          'purple'};
x_label = 'bend';
y_label = 'twist';
z_label = 'score';

sorted = {};
sorted_median = {};

for file_ind=1:length(files)
    all_data = load(files{file_ind});
    %to_plot = to_plot(find(to_plot(:,1) >= 0),:);
%{
    for row_ind=1:size(all_data,1)
        to_mean = all_data(row_ind,5:end).^2;
        all_data(row_ind,4) = sqrt(sum(to_mean));
    end
%}

    to_plot = all_data;

%{
    %heat plot
    figure;
    [x,y] = meshgrid(to_plot(:,1), to_plot(:,2));
    x = unique(x);
    y = unique(round(y*1e3)/1e3);
    Z = zeros(length(y), length(x));
    
    for i=1:size(to_plot,1)
        Z(find(abs(y - to_plot(i,2))<1e-3), find(abs(x - to_plot(i,1)) < 1e-3)) = to_plot(i,4);
    end
    surf(x,y,Z);
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title(titles{file_ind});
%}
    
    sorted{file_ind} = sortrows(to_plot,4);
    sorted{file_ind}(:,5) = sorted{file_ind}(:,2)./sorted{file_ind}(:,1);
    
    figure;
    scatter3(sorted{file_ind}(1:1000,1), sorted{file_ind}(1:1000,2),sorted{file_ind}(1:1000,4),'filled');
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title(titles{file_ind}); 

   




    %plot everything for median
    for_median = to_plot;
    for row_ind=1:size(for_median,1)
        for_median(row_ind,4) = median(to_plot(row_ind,5:end));
    end
%{
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

    %}
    
   





    sorted_median{file_ind} = sortrows(for_median,4);
    %{
    figure;
    scatter3(sorted_median{file_ind}(1:1000,1), sorted_median{file_ind}(1:1000,2),sorted_median{file_ind}(1:1000,3),'filled');
    xlabel(x_label);
    ylabel(y_label);
    zlabel(z_label);
    title([titles{file_ind} ' median']);
    %}


    
    %}
end


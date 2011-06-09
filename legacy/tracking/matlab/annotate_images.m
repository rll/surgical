function pixels = annotate_images(numbers)
%% debug
if strcmp(getenv('OSTYPE'),'linux')
    directory = '/windows/D/Research/modelbasedtracking';
else
    directory = 'D:\\Research\\modelbasedtracking';
end
imDir = 'data/2010Jun18';
pre = 'hand';

    
pixels = cell(1,3);
for k=1:3
    pixLists = cell(1,length(numbers));
    for i=1:length(numbers)
        imNum = numbers(i);
        filename = [directory '/' imDir '/' pre int2str(k) '-' int2str(imNum) '.tiff']
    	pixLists{i} = imageProcessorFast(filename, true);
%      	pixLists{i} = imageProcessorFast_trace(filename);
    end
    pixels{k} = cat(1, pixLists{:});
    k_avgs = mean(pixels{k}, 1);
end

end
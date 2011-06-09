imDir = 'data/2010Jul01/rect/BandOnly/';
directory = HomeDirectory;
pre = 'warped_test';
k = 1;
nums = 406:492;

nlevels = 3;
nrows = sum((4*ones(1,nlevels)) .^ (2:(nlevels+1)));
hogDescriptors = zeros(nrows, length(nums));

for i=1:length(nums)
    imNum = nums(i);
    im = imread([directory imDir pre num2str(k) '-' num2str(imNum) '.ppm']);
    hogDesc = compute_sp_hog(im, 8, nlevels);
    hogDescriptors(:, i) = hogDesc/norm(hogDesc);
end


%% analysis
dists = hogDescriptors' * hogDescriptors;

%% different hog descriptor

imDir = 'data/2010Jul01/rect/BandOnly/';
directory = HomeDirectory;
pre = 'warped_test';
k = 1;
nums = 406:492;

nlevels = 3;
nbins = 8;

first = true;
for i=1:length(nums)
    imNum = nums(i);
    filepath = [directory imDir pre num2str(k) '-' num2str(imNum) '.ppm'];
    desc = jinna_phog(filepath, nbins, 360, nlevels);
    if first
        HogDescriptors = zeros(length(desc), length(nums));
        first = false;
    end
    HogDescriptors(:,i) = desc;
end    
    
%% analysis
load gross_rolls.txt

trainNums = nums(1:floor(length(nums)/2));
testNums = nums(ceil(length(nums)/2):end);
trainDescs = HogDescriptors(:, ismember(nums, trainNums));
testDescs = HogDescriptors(:, ismember(nums, testNums));

regress_rolls = zeros(size(testDescs,2), 1);
real_rolls = gross_rolls(ismember(nums, testNums));
for i=1:size(testDescs,2)
    descDiffs = vadd(trainDescs, -testDescs(:,i));
    diffNorms = normArray(descDiffs, 1);
    [m, mini] = min(diffNorms);
    regress_rolls(i) = gross_rolls(mini);
end


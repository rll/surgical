%% find gross rolls
%% First, calculate a bunch of descriptors. 

imDir = 'data/2010Jul01/rect/BandOnly/';
directory = HomeDirectory;
pre = 'warped_test';
k = 1;
nums = 406:492;

nlevels = 3;
nbins = 16;

first = true;
for i=1:length(nums)
    imNum = nums(i);
    %%
    filepath = [directory imDir pre num2str(k) '-' num2str(imNum) '.ppm'];
    im = imread(filepath);
    desc = compute_sp_hog(im, nbins, nlevels, .02);
    if first
        HogDescriptors = zeros(length(desc), length(nums));
        first = false;
    end
    HogDescriptors(:,i) = desc;
end    
    
%% test
%% gross_rolls.txt is "ground truth." 
%% Split the descriptors calculated above into test and training sets. We
%% pretend we don't know the ground truth for the test set, and
%% calculate the best roll for each based on their similarities to the
%% training vectors. 

load gross_rolls.txt
gross_rolls = mod(gross_rolls, 180);

trainNums = nums(1:floor(length(nums)/2));
testNums = nums(ceil(length(nums)/2):end);
trainDescs = HogDescriptors(:, ismember(nums, trainNums));
testDescs = HogDescriptors(:, ismember(nums, testNums));

train_rolls = gross_rolls(ismember(nums, trainNums));

regress_rolls = zeros(size(testDescs,2), 1);
real_rolls = gross_rolls(ismember(nums, testNums));

nk = 3;
for i=1:size(testDescs,2)
    descDiffs = vadd(trainDescs, -testDescs(:,i));
    diffNorms = normArray(descDiffs, 1);
%     diffNorms = sum(abs(descDiffs), 1);
    
    [sDiffNorms, reorder] = sort(diffNorms);
    angs = train_rolls(reorder(1:nk));
%     weights = (2*ones(1,nk)).^(-sDiffNorms(1:nk));
    weights = ones(1,nk)./(sDiffNorms(1:nk).^2);
    weights = weights/max(weights);
    weights = weights';

    angs2 = angs;
    boss = angs(1);
    % we can't tell theta from theta+180. 
    bool = abs(angs-boss) > 90;
    angs2(bool) = angs(bool) + 180*sign(boss - angs(bool)) ;
    
    ma = sum(cos(angs2*pi/180) .* weights)/sum(weights);
    mb = sum(sin(angs2*pi/180) .* weights)/sum(weights);
    
    mang = atan2(mb, ma)*180/pi;
    conf = norm([ma mb]);
    
    regress_rolls(i) = mang;
    confs(i) = conf;
end

%% Output stats. 
adiffs = mod(real_rolls - regress_rolls, 180);
ad = min(adiffs, 180-adiffs);                   % difference to ground truth
disp('******************')
avg = mean(ad)
dev = std(ad)
mx = max(ad)
avg2 = mean(ad(ad < 10))
dev2 = std(ad(ad<10))
n = sum(ad<10)

% Main function for running experiments on the Caltech-4 dataset.
% This function:
%
% (1) Reads 20 training and 20 test images for each of the
%     four object classes (airplanes, Faces, Leopards, Motorbikes)
%
% (2) Computes a descriptor for each image.
%     (You will need to implement this part)
%
% (3) Trains a multi-class SVM, reports classification accuracy
%     on the test set, and displays a class-confusion matrix.
%
% (4) Returns the test accuracy and confusion matrix.
%
% Three input arguments may be specified:
%
%   n_ori_bins - the number of orientations to use when quantizing
%                gradient features in the descriptor computation
%
%   n_levels   - the number of levels of the spatial pyramid
%
%   C          - the SVM error penalty parameter
function [accu table] = main(n_ori_bins, n_levels, C)

% choose some reasonable default arguments
if (nargin < 1)
    n_ori_bins = 8;   % gradients quantized into 8 orientations
end

if (nargin < 2)
    n_levels = 6     % 5 levels in spatial pyramid
end

if (nargin < 3)
    C = 1000;         % SVM error penalty parameter
end

% categories for training/testing
categs = {'airplanes','Faces','Leopards','Motorbikes'};

% assemble training data
train_instances = [];
train_labels = [];
for cc = 1:length(categs),
    categ  = categs{cc};
    dd = dir(['data/train/' categ '/*.jpg']);
    for id = 1:length(dd),            
        % load image
        I = imread(['data/train/' categ '/' dd(id).name]);
        % compute spatial pyramid descriptor
        descriptor = compute_sp_hog(I,n_ori_bins,n_levels);
        % store test instance
        train_instances = [train_instances; descriptor'];
        train_labels = [train_labels; cc];
    end
    fprintf('Loading training data and computing descriptors for category %s.\n',categ);
end

% assemble test data
test_instances = [];
test_labels = [];
for cc = 1:length(categs),
    categ  = categs{cc};
    dd = dir(['data/test/' categ '/*.jpg']);
    for id = 1:length(dd),            
        % load image
        I = imread(['data/test/' categ '/' dd(id).name]);
        % compute spatial pyramid descriptor
        descriptor = compute_sp_hog(I,n_ori_bins,n_levels);
        % store test instance
        test_instances = [test_instances; descriptor'];
        test_labels = [test_labels; cc];
    end
    fprintf('Loading test data and computing descriptors for category %s.\n',categ);
end

% add libsvm to matlab path
addpath('libsvm-mat-2.84-1');

% perform SVM classification
disp('******************** Running SVM ********************');
isonevsone = false;
if isonevsone,
    % multi-class SVM (one-vs-one)
    model = svmtrain(train_labels, train_instances, ['-t 0 -c ' num2str(C) ' -b 1']);
    test_predict = svmpredict(test_labels, test_instances, model, '-b 1');
else
    % multi-class SVM (one-vs-all)
    pw = length(categs)-1;
    ptrain = zeros(length(train_labels),length(categs));
    ptest  = zeros(length(test_labels), length(categs));
    tic;
    for cc = 1:length(categs),

        label = (train_labels == cc) * 2 - 1;
        model = svmtrain(label, train_instances, ['-t 0 -c ' num2str(C) ' -b 1 -w1 ' num2str(pw)]);
        [pl, acc, ptr] = svmpredict(label, train_instances, model, '-b 1');

        label = (test_labels == cc) * 2 - 1;
        [pl, acc, pte] = svmpredict(label, test_instances, model, '-b 1');

        if model.Label(1) == 1,
            ptrain(:,cc) = ptr(:,1);
            ptest(:,cc) = pte(:,1);
        else
            ptrain(:,cc) = ptr(:,2);
            ptest(:,cc) = pte(:,2);
        end
        fprintf('Done SVM classification on category %d/%d in %g seconds.\n',cc,length(categs),toc);

    end
    [ignore,train_predict] = max(ptrain,[],2);
    [ignore,test_predict] = max(ptest,[],2);
end
disp('*****************************************************');

% display category confusion table and report accuracy
table = compute_confusion_table(test_labels, test_predict);
figure(1); imagesc(table); title('Category confusion matrix');
accu = mean(diag(table)) / 20;
disp(['Test Classification Accuracy = ' num2str(accu)]);

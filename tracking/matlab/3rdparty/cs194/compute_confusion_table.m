% Compute the confusion table from the actual labels and the predicted
% labels. sum(table,2) = vector of 1's.
function table = compute_confusion_table(tlabel, plabel)

uniqlabel = unique(tlabel);
uniqplabel = unique(plabel);
for ii = 1:length(uniqplabel),
    if ~any(~(uniqlabel - uniqplabel(ii))),
        error('Unknown predicted label!');
    end;
end;
%%% reassign labels
for ii = 1:length(uniqlabel),
    tlabel(tlabel==uniqlabel(ii))=ii;
    plabel(plabel==uniqlabel(ii))=ii;
end;

nc = length(uniqlabel);
table = zeros(nc);
for ii = 1:length(tlabel),
    table(tlabel(ii),plabel(ii)) = table(tlabel(ii),plabel(ii)) + 1;
end;

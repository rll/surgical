% Inputs

% obin -- obin(i,j) is the orientation bin of the (i,j)th pixel.
% mag -- mag(i,j) is the gradient magnitude of the (i,j)th pixel.
% thresh -- magnitude threshold, i.e. if a pixel has gradient magnitude
%     less than thresh, it is discarded.

% returns a histogram of the orientations in obin.

function hist = compute_histogram2(obin, mag, n_ori_bins, thresh)

    if nargin < 4
        thresh = -1;
    end
    hist = zeros(1, n_ori_bins);
    for i = 1:size(obin,1)
       for j = 1:size(obin,2)
           if mag(i,j) >= thresh
               hist(obin(i,j)) = hist(obin(i,j)) + 1;
           end
       end
    end    
%     'histogram before normalizing'
%     hist
    Z = (size(obin,1) * size(obin,2))
    hist = hist/(size(obin,1) * size(obin,2));        % normalize by number of elements
end

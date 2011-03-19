
function hist = compute_histogram(obin, mag, n_ori_bins)
    discard_thresh = 0;%0.4*max(max(mag));
    separation_thresh = 10000;%0.8*max(max(mag));
    
    hist = zeros(1, n_ori_bins*2);
    for i = 1:size(obin,1)
       for j = 1:size(obin,2)
           if (mag(i,j) > discard_thresh && mag(i,j) <= separation_thresh)
               hist(2*(obin(i,j)-1)+1) = hist(2*(obin(i,j)-1)+1) + 1;
           elseif (mag(i,j) > separation_thresh)
               hist(2*(obin(i,j)-1)+2) = hist(2*(obin(i,j)-1)+2) + 1;
           end
       end
    end
    hist = hist/(size(obin,1) * size(obin,2) + .000001);
end

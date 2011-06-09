% adds vector v to array of vectors arr

function b  = vadd(arr, v)

b = arr;
if size(arr,1) == numel(v)
    for i=1:size(arr,1)
        b(i,:) = arr(i,:) + v(i);
    end
elseif size(arr,2) == numel(v)
    for i=1:size(arr,2)
        b(:,i) = arr(:,i) + v(i);
    end
end
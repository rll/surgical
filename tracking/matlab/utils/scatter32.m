function scatter32(A)
sz = size(A);
if sz(2) == 3
    scatter3(A(:,1), A(:,2), A(:,3), '.');
elseif sz(1) == 3
    scatter3(A(1,:), A(2,:), A(3,:), '.');
end

end
    
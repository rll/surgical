load energy_eval_points.txt;
X = energy_eval_points(:,1:5);
Y = energy_eval_points(:,6);


K = zeros(35,31);
for i=1:size(X,1)
   x = X(i,:);
   K(i,:) = [reshape(x'*x,1,25) x 1];
end

vals = lscov(K,Y);

hessian = reshape(vals(1:25),5,5);
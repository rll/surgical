function [ ] = solve_sparse(A_m, A_n, A_file, b_m, b_n, b_file, x_file)
% Solve Sparse is a script that loads A and B and solves for Ax = b.
% Writes x to x_file 

A_data = load(A_file);
A = sparse(A_data(:, 1), A_data(:, 2), A_data(:, 3), A_m, A_n);
%A = A_data;

b_data = load(b_file);
b = sparse(b_data(:, 1), b_data(:, 2), b_data(:, 3), b_m, b_n); 
%b = b_data;
x = A \ b; 

dlmwrite(x_file, full(x), 'precision', 10); 
exit();
end


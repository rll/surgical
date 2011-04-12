function [] = solve_sparse(A_m, A_n, A_file, b_m, b_n, b_file, x_file, num_threads, size_each_state, size_each_control) 
% Solve Sparse is a script that loads A and B and solves for Ax = b.
% Writes x to x_file 

A_data = load(A_file);
A = sparse(A_data(:, 1), A_data(:, 2), A_data(:, 3), A_m, A_n);
%A = A_data;

b_data = load(b_file);
%b = sparse(b_data(:, 1), b_data(:, 2), b_data(:, 3), b_m, b_n); 
b = load(b_file);

weight_vector = zeros(size_each_state,1);
for i=1:size_each_state
  weight_vector(i) = -A(i,i);
end

b = zeros(size_each_state*(num_threads-1),1);
goal_state = b(end-size_each_state+1: end);
goal_state = repmat(goal_state, num_threads-2, 1);
b(1:size_each_state) = b_data(1:size_each_state) .* -weight_vector;
b(end-size_each_state+1: end) = b_data(end-size_each_state+1:end) .* weight_vector;

weighted_state_diff_constraint = 50;
control_diff_constraint = 1;

cvx_solver sdpt3

cvx_begin
    variable x(A_n)
    minimize (norm(A*x - b) + 0.1*norm(x((num_threads-2)*size_each_state:end)) + 0*norm(goal_state-x(1:length(goal_state))));
  
    subject to
      for thread_num=1:num_threads-2
        norm( weight_vector.* x(size_each_state*(thread_num-1) + 1: size_each_state*(thread_num)) - weight_vector.* b_data((thread_num*size_each_state)+1 : (thread_num+1)*size_each_state)) < weighted_state_diff_constraint;
      end
      
      for thread_num=1:num_threads-1
        norm( x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control)) < control_diff_constraint;
      end
    

      
cvx_end

dlmwrite(x_file, full(x), 'precision', 10); 
exit();
end


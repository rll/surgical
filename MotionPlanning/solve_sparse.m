function [] = solve_sparse(A_m, A_n, A_file, b_m, b_n, b_file, x_file, num_threads, size_each_state, size_each_control) 
% Solve Sparse is a script that loads A and B and solves for Ax = b.
% Writes x to x_file 

A_data = load(A_file);
A = sparse(A_data(:, 1), A_data(:, 2), A_data(:, 3), A_m, A_n);

b_data = load(b_file);

weight_vector = zeros(size_each_state,1);
for i=1:size_each_state
  weight_vector(i) = -A(i,i);
end

b = zeros(size_each_state*(num_threads-1),1);
goal_state = b(end-size_each_state+1: end);
goal_state = repmat(goal_state, num_threads-2, 1);
b(1:size_each_state) = b_data(1:size_each_state) .* -weight_vector;
b(end-size_each_state+1: end) = b_data(end-size_each_state+1:end) .* weight_vector;

weighted_state_diff_constraint = 5;
control_diff_constraint = 1;
min_control = 0.1;
%consecutive_state_diff_constraint = 10;

consectuve_state_diff_weight = zeros(100,1);

cvx_solver sdpt3
cvx_begin
    variable x(A_n)
    variable u((num_threads-1))
    variable consecutive_state_diff(num_threads)
    %minimize (square_pos(norm(A*x - b)) + 2*sum(sum(reshape(x((num_threads-2)*size_each_state+1:end), num_threads-1, size_each_control).^2,2)));
  
    minimize(norm(A*x-b) + norm(u, 1) + consectuve_state_diff_weight'*consecutive_state_diff)%norm(consecutive_state_diff,1)) %sum(w)
    subject to

      %% b_data contains all the state info from prev iteration
      for thread_num=1:num_threads-2
        norm( weight_vector.* x(size_each_state*(thread_num-1) + 1: size_each_state*(thread_num)) - weight_vector.* b_data((thread_num*size_each_state)+1 : (thread_num+1)*size_each_state)) < weighted_state_diff_constraint;
      end
      
      for thread_num = 1:num_threads-2
        consecutive_state_diff(thread_num+1) > norm(weight_vector .* x(size_each_state*(thread_num-1) + 1 : size_each_state*(thread_num)) - weight_vector .* x(size_each_state*(thread_num) + 1 : size_each_state*(thread_num+1)));
      end

      consecutive_state_diff(1) > norm(weight_vector .* x(1:size_each_state) - weight_vector .* -b(1:size_each_state));

      consecutive_state_diff(num_threads) > norm(weight_vector .* x(size_each_state*(num_threads-1) + 1 : size_each_state*(num_threads)) - weight_vector .* b(end-size_each_state+1:end)); 

      %consecutive_state_diff(1:end) < consecutive_state_diff_constraint;

      for thread_num = 1:num_threads-2
        if mod(thread_num, 1) ~= 0 
          x(size_each_state*(num_threads-2)+(thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control) == x(size_each_state*(num_threads-2)+(thread_num)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num+1)*size_each_control);
        end
      end
      
      %for thread_num=1:num_threads-1
       % norm( x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control)) < control_diff_constraint;
      %end
      for thread_num=1:num_threads-1
        u(thread_num) >= norm( x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control));
      end
      u(1:end) < control_diff_constraint;
      u(1:end) > min_control;


cvx_end

tmp = full(x);
reshape(tmp(size_each_state*(num_threads-2)+1:end)', 12, [])


dlmwrite(x_file, full(x), 'precision', 10); 
exit();
end


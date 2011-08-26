function [] = solve_sparse(A_m, A_n, A_file, b_m, b_n, b_file, x_file, num_threads, size_each_state, size_each_control) 
% Solve Sparse is a script that loads A and B and solves for Ax = b.
% Writes x to x_file 

edge_length = 3;

A_data = load(A_file);
A = sparse(A_data(:, 1), A_data(:, 2), A_data(:, 3), A_m, A_n);
J_diag_block = full(A(:, size_each_state*(num_threads-2)+1:end));
J_block = zeros(size_each_state, size_each_control*(num_threads-1));

for i = 1:num_threads-1
  J_block(1:end,size_each_control*(i-1)+1:size_each_control*i) = J_diag_block(size_each_state*(i-1)+1:size_each_state*i, size_each_control*(i-1)+1:size_each_control*i);
end

b_data = load(b_file);

weight_vector = zeros(size_each_state,1);
for i=1:size_each_state
  weight_vector(i) = -A(i,i);
end

b = zeros(size_each_state*(num_threads-1),1);
b(1:size_each_state) = b_data(1:size_each_state) .* -weight_vector;
b(end-size_each_state+1: end) = b_data(end-size_each_state+1:end) .* weight_vector;
initial_state = -b(1:size_each_state);
goal_state = b(end-size_each_state+1: end);

%num_points = (size_each_state-1)/3;

weighted_state_diff_constraint = 0.1;
control_constraint = 5e-2;

%min_control = 0.1;
%consecutive_state_diff_constraint = 10;

consectuve_state_diff_weight = zeros(100,1);

cvx_solve sdpt3
cvx_begin
    variable x(A_n)
    variable u((num_threads-1))
    %variable consecutive_state_diff(num_threads)
    %variable edge_violations((num_threads-2)*num_points)
    variable consecutive_control_diff((num_threads-2)*size_each_control)
    variable u_block(size_each_control*(num_threads-1))
    variable forward_scores((num_threads-1)*size_each_state)
    %minimize (square_pos(norm(A*x - b)) + 2*sum(sum(reshape(x((num_threads-2)*size_each_state+1:end), num_threads-1, size_each_control).^2,2)));
  
    %minimize(norm(A*x-b))
    %minimize(norm(A*x-b))

    %minimize(norm(forward_scores) + 0.0001 * norm(u_block,1))
    minimize(norm(forward_scores) + 0.05 * norm(u_block,1))




    %minimize(norm(A*x-b) + norm(edge_violations,2) + norm(u,2)) %+ norm(consecutive_control_diff, 2) + norm(u,2) + norm(forward_scores)) %+ consectuve_state_diff_weight'*consecutive_state_diff)%norm(consecutive_state_diff,1)) %sum(w)
    subject to

      %norm(A*x-b) < 0.05
     
     %for j = 1:num_threads-2
     %   forward_scores(size_each_state*(j-1)+1:size_each_state*j) == J_block(:,1:size_each_control*j)*u_block(1:size_each_control*j) + initial_state - x(size_each_state*(j-1)+1 : size_each_state*j); 
     %   forward_scores(size_each_state*(j-1)+1:size_each_state*j) == J_block(:,1:size_each_control*j)*u_block(1:size_each_control*j) + initial_state - goal_state;
      %J_block(:,1:size_each_control*j)*u_block(1:size_each_control*j) + -initial_state - b_data(j*size_each_state + 1 : (j+1)*size_each_state) < weighted_state_diff_constraint;
     %end


      forward_scores((num_threads-2)*size_each_state+1:end) == J_block*u_block + initial_state - goal_state;
%TODO: CHANGE EDGE VIOLATIONS TO BE REST LENGTH
      %for thread_num=1:num_threads-2
      %  for vertex_num = 1:num_points-1
      %    norm(x(size_each_state*(thread_num-1) + 3*(vertex_num-1)+1 : size_each_state*(thread_num-1) + 3*vertex_num) - x(size_each_state*(thread_num-1) + 3*(vertex_num)+1 : size_each_state*(thread_num-1)+3*(vertex_num+1))) < edge_violations((thread_num-1)*(num_points-1)+vertex_num) + edge_length
      %  end
      %end

      %% b_data contains all the state info from prev iteration
      %for thread_num=1:num_threads-2
      %  weight_vector.* x(size_each_state*(thread_num-1) + 1: size_each_state*(thread_num)) - weight_vector.* b_data((thread_num*size_each_state)+1 : (thread_num+1)*size_each_state) < weighted_state_diff_constraint;
      %end
      
      %for thread_num = 1:num_threads-2
      %  consecutive_state_diff(thread_num+1) > norm(weight_vector .* x(size_each_state*(thread_num-1) + 1 : size_each_state*(thread_num)) - weight_vector .* x(size_each_state*(thread_num) + 1 : size_each_state*(thread_num+1)));
      %end

      %consecutive_state_diff(1) > norm(weight_vector .* x(1:size_each_state) - weight_vector .* -b(1:size_each_state));

      %consecutive_state_diff(num_threads) > norm(weight_vector .* x(size_each_state*(num_threads-1) + 1 : size_each_state*(num_threads)) - weight_vector .* b(end-size_each_state+1:end)); 

      %consecutive_state_diff(1:end) < consecutive_state_diff_constraint;

      %for thread_num = 1:num_threads-2
      %  if mod(thread_num, 1) ~= 0 
      %    x(size_each_state*(num_threads-2)+(thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control) == x(size_each_state*(num_threads-2)+(thread_num)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num+1)*size_each_control);
      %  end
      %end
      
      %for thread_num=1:num_threads-1
       % norm( x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control)) < control_diff_constraint;
      %end
      for thread_num=1:num_threads-1
        u(thread_num) >= norm( x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control));
        %abs(x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +4 : size_each_state*(num_threads-2) + (thread_num-1)*size_each_control + 6)) < 1e-2;
        %abs(x(size_each_state*(num_threads-2) + (thread_num-1)*size_each_control +10 : size_each_state*(num_threads-2) + (thread_num-1)*size_each_control + 12)) < 1e-2;
      end
      %u(1:end) < control_constraint;

      for thread_num=1:num_threads-2
        consecutive_control_diff(size_each_control*(thread_num-1) + 1 : size_each_control*thread_num) == x(size_each_state*(num_threads-2)+(thread_num-1)*size_each_control+1 : size_each_state*(num_threads-2) + (thread_num)*size_each_control) - x(size_each_state*(num_threads-2)+thread_num*size_each_control+1 : size_each_state*(num_threads-2) + (thread_num+1)*size_each_control)
     end

      u_block(1:end) == x(size_each_state*(num_threads-2)+1:end)
      %for thread_num=1:num_threads-1
      %  u_block(6+(thread_num-1)*size_each_control + 1 : thread_num*size_each_control) == 0; 
      %end
      abs(u_block(1:end)) < control_constraint; 
      %abs(x(size_each_state*(num_threads-2)+1:end)) < control_constraint;
      %u(1:end) > min_control;


cvx_end


tmp = full(x);
reshape(tmp(size_each_state*(num_threads-2)+1:end)', 12, [])


dlmwrite(x_file, full(x), 'precision', 10); 
exit();
end


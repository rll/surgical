clear all;

start = load('saved_starts_19.txt');
goal = load('saved_goals_19.txt');
start_angles = start(:,90);
goal_angles = goal(:,90);

start_pts = zeros(19, 3);
end_pts = zeros(19, 3);

point_diff_norm = zeros(250, 1);
start_first_last_diff_norm = zeros(250,1);
end_first_last_diff_norm = zeros(250,1);

for k = 1:250
    for i = 1:19
        for j = 1:3
            start_pts(i,j) = start(k, 18+4*(i-1)+j);
            goal_pts(i,j) = goal(k, 18+4*(i-1)+j);
        end
    end
    
    point_diff_norm(k,1) = norm(sqrt(sum((start_pts - goal_pts).^2,2))); 
    start_first_last_diff_norm(k,1) = sqrt(norm(start_pts(1,:) - start_pts(end,:))^2);
    end_first_last_diff_norm(k,1) = sqrt(norm(goal_pts(1,:) - goal_pts(end,:))^2);
end



closed_loop_sqp = load('CLOSED_LOOP_SQP_19_results.txt');
linearize_only = load('LINEARIZE_ONLY_19_results.txt');
linearize_via_trajectory = load('LINEARIZE_VIA_TRAJECTORY_19_results.txt');
open_loop_sqp = load('OPEN_LOOP_SQP_19_results.txt');
rrt_planner = load('RRT_PLANNER_19_results.txt');
rrt_planner_sqp_closed_loop_smoother = load('RRT_PLANNER_SQP_CLOSEDLOOP_SMOOTHER_19_results.txt');
rrt_planner_sqp_open_loop_smoother = load('RRT_PLANNER_SQP_OPENLOOP_SMOOTHER_19_results.txt');
rrt_planner_sqp_closedloop_to_end = load('RRT_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST_19_results.txt');
rrt_dim1_planner = load('RRT_dim1_PLANNER_19_results.txt');
rrt_dim1_planner_sqp_closed_loop_smoother = load('RRT_dim1_PLANNER_SQP_CLOSEDLOOP_SMOOTHER_19_results.txt');
rrt_dim1_planner_sqp_open_loop_smoother = load('RRT_dim1_PLANNER_SQP_OPENLOOP_SMOOTHER_19_results.txt');
rrt_dim1_planner_sqp_closedloop_to_end = load('RRT_dim1_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST_19_results.txt');
%rrt_dim2_planner = load('RRT_dim2_PLANNER_19_results.txt');
%rrt_dim2_planner_sqp_closed_loop_smoother = load('RRT_dim2_PLANNER_SQP_CLOSEDLOOP_SMOOTHER_19_results.txt');
%rrt_dim2_planner_sqp_open_loop_smoother = load('RRT_dim2_PLANNER_SQP_OPENLOOP_SMOOTHER_19_results.txt');
%rrt_dim2_planner_sqp_closedloop_to_end =
%load('RRT_dim2_PLANNER_SQP_CLOSEDLOOP_ONLY_LAST_19_results.txt');

scores = [linearize_only(:,1), linearize_via_trajectory(:,1), open_loop_sqp(:,1), closed_loop_sqp(:,1), rrt_planner(:,1), rrt_planner_sqp_open_loop_smoother(:,1), rrt_planner_sqp_closed_loop_smoother(:,1), rrt_planner_sqp_closedloop_to_end(:,1) ];
scores = [scores, rrt_dim1_planner(:,1), rrt_dim1_planner_sqp_closed_loop_smoother(:,1), rrt_dim1_planner_sqp_open_loop_smoother(:,1), rrt_dim1_planner_sqp_closedloop_to_end(:,1)];
%scores = [scores, rrt_dim2_planner(:,1), rrt_dim2_planner_sqp_closed_loop_smoother(:,1), rrt_dim2_planner_sqp_open_loop_smoother(:,1), rrt_dim2_planner_sqp_closedloop_to_end(:,1)];
times = [linearize_only(:,2), linearize_via_trajectory(:,2), open_loop_sqp(:,2), closed_loop_sqp(:,2), rrt_planner(:,2), rrt_planner_sqp_open_loop_smoother(:,2), rrt_planner_sqp_closed_loop_smoother(:,2), rrt_planner_sqp_closedloop_to_end(:,2) ];
times  = [times, rrt_dim1_planner(:,2), rrt_dim1_planner_sqp_closed_loop_smoother(:,2), rrt_dim1_planner_sqp_open_loop_smoother(:,2), rrt_dim1_planner_sqp_closedloop_to_end(:,2)];
%times = [times, rrt_dim2_planner(:,2), rrt_dim2_planner_sqp_closed_loop_smoother(:,2), rrt_dim2_planner_sqp_open_loop_smoother(:,2), rrt_dim2_planner_sqp_closedloop_to_end(:,2)];
table = [mean(scores(1:250, :)/18); std(scores(1:250, :)/18); mean(times(1:250, :))]




text = ['lt ', 'clsqp ', 'rrt ', 'rrtclsqp ', 'rrtclsqpend ', 'rrt_dim1 ', 'rrt_dim1_clsqp ', 'rrt_dim1_clsqpend ', 'rrt_dim2 ', 'rrt_dim2_clsqp ', 'rrt_dim2_clsqpend ']
good_scores = [linearize_via_trajectory(:,1), closed_loop_sqp(:,1), rrt_planner(:,1), rrt_planner_sqp_closed_loop_smoother(:,1), rrt_planner_sqp_closedloop_to_end(:,1)];
good_scores = [good_scores, rrt_dim1_planner(:,1), rrt_dim1_planner_sqp_closed_loop_smoother(:,1), rrt_dim1_planner_sqp_closedloop_to_end(:,1)];
%good_scores = [good_scores, rrt_dim2_planner(:,1), rrt_dim2_planner_sqp_closed_loop_smoother(:,1), rrt_dim2_planner_sqp_closedloop_to_end(:,1)];


plot_1_scores = [linearize_via_trajectory(:,1), closed_loop_sqp(:,1), rrt_planner(:,1), rrt_planner_sqp_closed_loop_smoother(:,1) rrt_planner_sqp_closedloop_to_end(:,1)];
plot_1_scores = plot_1_scores(1:250, :);

mean(good_scores)
std(good_scores)
h = figure;
hold on;

colorSet = varyColor(size(plot_1_scores,2));

for i = 1:size(plot_1_scores,2)
    bettercdfplot(plot_1_scores(:,i)/18, colorSet(i,:));
end
legend('Local Control Along Interpolated Trajectory', 'LCSQP', 'RRT Planner', 'RRT Planner with LCSQP Smoother', 'RRT Planner with LCSQP to End', 'Location', 'Southeast' );  
xlabel('Distance from Goal', 'FontSize',20)
ylabel('Fraction Successful','FontSize',20)
xlim([0,1.5])
set(findobj('type','axes'),'FontSize',16);

plot_2_scores = [rrt_planner(:,1), rrt_planner_sqp_closedloop_to_end(:,1), rrt_dim1_planner(:,1), rrt_dim1_planner_sqp_closedloop_to_end(:,1)];
%print(h, '-dpdf', '~/rll-svn/papers/trunk/2011_ISRR_threadplanning/19_scores.pdf', '-r125');

h = figure;
hold on;

colorSet = varyColor(size(plot_2_scores,2));

for i = 1:size(plot_2_scores,2)
    bettercdfplot(plot_2_scores(:,i)/34, colorSet(i,:));
end
legend('RRT Planner', 'RRT Planner with LCSQP to End', 'DR Planner (n = 1)', 'DR Planner with LCSQP to End (n = 1)', 'Location', 'Southeast' );  
xlabel('(Distance from Goal)/(Number of Edges)', 'FontSize', 20)
ylabel('Fraction Complete', 'FontSize', 20)
xlim([0,1.5])
set(findobj('type','axes'),'FontSize',16);
%print(h, '-dpdf', '~/rll-svn/papers/trunk/2011_ISRR_threadplanning/19_dr_scores.pdf', '-r125');





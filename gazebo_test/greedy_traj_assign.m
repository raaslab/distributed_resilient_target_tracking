% greedy target assign algorithm
% 1. calculate the number of targets tracked by the centralized greedy algorithm and
% the number of targets after best removal
% 2. assign each uav a greedy trajectory
% 3. publish desired pos message for each uav(same as 2. assign trajectory)
function greedy_traj_assign(target_cover)
            
    global N_uavs

    global uav_id_set

    tic; 

    traj_assign = zeros(N_uavs,1); % each uav has a assigned traj for publisher control

    % centralized greedy
    [r_tra_assign] = greedy_fun(uav_id_set, [], target_cover); 

    % store the time of running the algorithm
    t_run = toc; 

    % calculate the communication
    com = nchoosek(uav_id_set,2);

    % give value from r_tra_assign to traj_assign for publisher 
    for i = 1 : N_uavs
        traj_assign(r_tra_assign(i,1)) = r_tra_assign(i,2); 
    end     

    % after getting the trajectories for all the robots, tested by worst attack
    [n_after_atk]= worst_attack(target_cover, r_tra_assign);
    
    % here we give a fake cliq id for the publisher 
    cliq_id  = zeros(N_uavs, 1);

    %desired pos_publish, keep publishing desired for each uav
    %collect the targets tracked after removal, t_run, communication
    desired_pos_publisher_cen(traj_assign, n_after_atk, cliq_id, t_run, com);                 
end
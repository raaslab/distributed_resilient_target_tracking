function     [com_cen_resi, t_cen_resi] = ...
        cen_resi_fun(robot_set_G, tar_set_rtra, n_tars_max_rtra, max_rtra_inx)
    tic;
    % perform an centralized oblivious greedy algorithm in the collected local biggest
    % alpha ones in central server 
    [r_tra_resi_cen, ~, eva_cen_resi] = ...
        oblivious_greedy_fun(robot_set_G, tar_set_rtra, n_tars_max_rtra, max_rtra_inx); 

    % store the time of running the algorithm
    t_cen_resi = toc; 
%     % calculate the targets tracked
%     [n_cen_resi] = n_tra_cover(tar_set_rtra, r_tra_resi_cen); 
%     % after getting the trajectories for all the robots, tested by worst attack
%     [n_cen_resi_atk]= worst_attack(tar_set_rtra, r_tra_resi_cen);
%     % worst attack rate
%      worst_att_rate_cen_resi = n_cen_resi_atk/n_cen_resi;
    % calculate the communication
    com_cen_resi = nchoosek(length(robot_set_G),2);
end
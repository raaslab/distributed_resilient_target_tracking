function  [com_cen_gre, t_cen_gre] ...
    = cen_gre_fun(robot_set_G, tar_set_rtra)
    tic;
    % perform an centralized oblivious greedy algorithm in the collected local biggest
    % alpha ones in central server 
    [r_tra_gre_cen, eva_cen_gre] = greedy_fun(robot_set_G, [], tar_set_rtra); 
    
    % store the time of running the algorithm
    t_cen_gre = toc;     
%     % calculate the targets tracked
%     [n_cen_gre] = n_tra_cover(tar_set_rtra, r_tra_gre_cen); 
%     % after getting the trajectories for all the robots, tested by worst attack
%     [n_cen_gre_atk]= worst_attack(tar_set_rtra, r_tra_gre_cen);
%     % worst attack rate
%      worst_att_rate_cen_gre = n_cen_gre_atk/n_cen_gre;
    % calculate the communication
    com_cen_gre = nchoosek(length(robot_set_G),2);
end
% resilient target assign algorithm
% 1. calculate the number of targets tracked by the distribtued resilient algorithm and
% the number of targets after best removal
% 2. assign each uav an resilient trajectory in each clique
% 3. publish desired pos message for each uav(same as 2. assign trajectory)
function distri_resi_traj_assign(target_cover, n_id_maxtra)

    global N_uavs 
    
    global uavs_pos
    
    global com_messages t_cliq_form
    
    % first step, clique formulation
    % find all the non-overlapping maximal cliques, we only need x and y
    % coordinations
    [nonovlap_cliqs_G, n_cliqs, cliq_num, cliq_id, com_messages, t_cliq_form] = ...
        nonoverlap_cliqs_fun(uavs_pos(:, 1:2));
    
    %keep running time and communication after cliq formulation
    tic; 
    
    % each uav has a assigned traj for publisher control   
    traj_assign = zeros(N_uavs,1); 
    
    % store the assignment.
    r_tra_assign = [];      
     
    % the communication in each clique 
    com_in_cliq = zeros(1, length(nonovlap_cliqs_G));    
    % for each clique do a resilient (an oblivous-greedy) algorithm
    for i = 1 : length(nonovlap_cliqs_G)
        %store the assignment for the robots in each clique
        [r_tra_resi_c, ~]= oblivious_greedy_fun(nonovlap_cliqs_G{i}, target_cover, n_id_maxtra); 
        r_tra_assign = [r_tra_assign; r_tra_resi_c]; 
        if length(nonovlap_cliqs_G{i}) > 1
            com_in_cliq(i) = nchoosek(length(nonovlap_cliqs_G{i}),2);
        else
            com_in_cliq(i) = 0;
        end
    end
    
    
    % store the running time of the algorithm
    t_run = toc/length(nonovlap_cliqs_G) + t_cliq_form;    
    
    % calculate the communication
    com = max(com_in_cliq);
    
    % give value from r_tra_assign to traj_assign for publisher 
    for i = 1 : N_uavs
        traj_assign(r_tra_assign(i,1)) = r_tra_assign(i,2); 
    end    
    
%     % calculate the targets tracked
%     [n_resi] = n_tra_cover(target_cover, r_tra_assign); 
    
    % after getting the trajectories for all the robots, tested by worst attack
    [n_after_atk]= worst_attack(target_cover, r_tra_assign);
    
%     % worst attack rate
%     worst_atk_rate_resi = n_resi_after_atk/n_resi;     
        

  %desired pos_publish, keep publishing desired for each uav, cliq_id
  %collect the targets tracked after removal
  % n_cliqs, cliq_num, t_run, communication
   desired_pos_publisher_dis(traj_assign, cliq_id, n_after_atk, ...
       n_cliqs, cliq_num, t_run, com);
   
end
% fully distributed resilient target tracking
% perform an oblivious-greedy algorithm in each clique

global Nr N_tra Nt N_atk 

% the # of robots, 
Nr = 8;
% the # of directions
N_tra = 4; 
% the # of targets 
Nt = 50; 
% the # of worst-case attacks
N_atk = 2; 
% the postions of all robots
% the positions of all targets 
pos_range = 100;
r_pos=rand(Nr,2)*pos_range;
tar_pos=rand(Nt,2)*pos_range;

% obtain a set of targets tracked by each trajectory
[tar_set_rtra, ~, n_tars_max_rtra, max_rtra_inx] = robot_tra_cover_fun(r_pos, tar_pos);

% find all the non-overlapping maximal cliques
[nonovlap_cliqs_G, num_of_cliqs_G, cliq_num_G] = nonoverlap_cliqs_fun(r_pos);

% give the trajectory assignments for the robots in each clique
r_tra_resi = []; 
% for each clique do an oblivous-greedy algorithm
for i = 1 : length(nonovlap_cliqs_G)
    %store the assignment for the robots in each clique
    [r_tra_resi_c]= oblivious_greedy_fun(nonovlap_cliqs_G{i}, tar_set_rtra,...
                                            n_tars_max_rtra, max_rtra_inx); 
    r_tra_resi = [r_tra_resi; r_tra_resi_c]; 
end
% calculate the targets tracked
[n_resi] = n_tra_cover(tar_set_rtra, r_tra_resi); 
% after getting the trajectories for all the robots, tested by worst attack
[n_resi_after_att]= worst_attack(tar_set_rtra, r_tra_resi);
% worst attack rate
 worst_att_rate_resi = n_resi_after_att/n_resi;
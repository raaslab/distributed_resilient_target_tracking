% centralized greedy algorithm with worst-case attack

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
% define robot_set
robot_set_G = (1:N);

% obtain a set of targets tracked by each trajectory
[tar_set_rtra, ~, n_tars_max_rtra, max_rtra_inx] = robot_tra_cover_fun(r_pos, tar_pos);

% perform an centralized oblivious greedy algorithm in the collected local biggest
% alpha ones in central server 
r_tra_gre_cen = greedy_fun(robot_set_G, tar_set_rtra); 

% calculate the targets tracked
[n_cen_gre] = n_tra_cover(tar_set_rtra, r_tra_gre_cen); 
% after getting the trajectories for all the robots, tested by worst attack
[n_cen_gre_atk]= worst_attack(tar_set_rtra, r_tra_gre_cen);
% worst attack rate
 worst_att_rate_cen_gre = n_cen_gre_atk/n_cen_gre;
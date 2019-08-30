% fully distributed resilient target tracking
% perform an oblivious-greedy algorithm in each clique

% the number of robots, 
global Nr N_tar N_attack
Nr = 6;
N_tar = 50; 
N_attack = 3; 
% give the postions of all robots
r_pos = zeros(Nr,2);
r_pos(:, 1) = 100 * rand(Nr, 1);
r_pos(:, 2) = 100 * rand(Nr, 1);

% give the positions of all targets 
tar_pos = zeros(N_tar,2);
tar_pos(:, 1) = 100 * rand(N_tar, 1);
tar_pos(:, 2) = 100 * rand(N_tar, 1);

% obtain the targets tracked by each robot
tar_cover = robot_tra_cover_fun(r_pos, tar_pos);

% find all the non-overlapping maximal cliques
[~,~,nonoverlap_cliques] = neighbor_nonoverlap_cliques_fun(r_pos);
% give the trajectory assignments for the robots in each clique
r_tra_resi = []; 
% for each clique do an oblivous-greedy algorithm
for i = 1 : length(nonoverlap_cliques)
    %store the assignment for the robots in each clique
    [r_tra_resi_c]= oblivious_greedy_fun(nonoverlap_cliques{i}, tar_cover); 
    r_tra_resi = [r_tra_resi; r_tra_resi_c]; 
end
% calculate the targets tracked
[n_resi] =n_tra_cover(tar_cover, r_tra_resi); 
% after getting the trajectories for all the robots, tested by worst attack
[n_resi_att]= worst_attack(tar_cover, r_tra_resi);
% worst attack rate
 worst_att_rate_resi = n_resi_att/n_resi;

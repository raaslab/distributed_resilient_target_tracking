% hierarchical distributed resilient target tracking with a central server,
% perfroming an oblivious algorithm
% perform a local greedy algorithm in each remaining clique

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

% find all the non-overlapping maximal cliques
[nonovlap_cliqs_G, num_of_cliqs_G, cliq_num_G] = nonoverlap_cliqs_fun(r_pos);

% choose out the first alpha N_atk biggest ones by an oblivious algorithm
r_tra_o = oblivious_fun(robot_set_G, N_atk, n_tars_max_rtra, max_rtra_inx);
% atk_set is the first row of r_tra_o
atk_set = r_tra_o(:,1);

% calcuate the non-overlapping cliques of G2
nonovlap_cliqs_G2 = {}; 
% the # of robots in each cliq
each_cliq_num = []; 
for i = 1 : length(nonovlap_cliqs_G)
    temp_nonovlap_cliqs_G2 = setdiff(nonovlap_cliqs_G{i}, atk_set); 
    if ~isempty(temp_nonovlap_cliqs_G2)
        nonovlap_cliqs_G2 = [nonovlap_cliqs_G2; temp_nonovlap_cliqs_G2]; 
        each_cliq_num = [each_cliq_num, length(temp_nonovlap_cliqs_G2)];
    end
end
% clique number of G2
cliq_num_G2 = max(each_cliq_num);
% the number of cliques in G2
num_of_cliqs_G2 = length(nonovlap_cliqs_G2); 

% give the trajectory assignments for the robots in each clique of G2
r_tra_g = []; 
% for each clique do an oblivous-greedy algorithm
for i = 1 : length(nonovlap_cliqs_G2)
    %store the assignment for the robots in each clique
    [r_tra_each_g]= greedy_fun(nonovlap_cliqs_G2{i}, tar_set_rtra); 
    r_tra_g = [r_tra_g; r_tra_each_g]; 
end
% calculate the tra_asign
r_tra_hier_obli = [r_tra_o; r_tra_g]; 
% calculate the targets tracked
[n_hier_obli] =n_tra_cover(tar_set_rtra, r_tra_hier_obli); 
% after getting the trajectories for all the robots, tested by worst attack
[n_hier_obli_atk]= worst_attack(tar_set_rtra, r_tra_hier_obli);
% worst attack rate
 worst_att_rate_cen_o = n_hier_obli_atk/n_hier_obli;
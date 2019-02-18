% hierarchical distributed resilient target tracking with a central server,
% perfroming an oblivious_greedy algorithm
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
robot_set = (1:N);

% obtain a set of targets tracked by each trajectory
[tar_set_rtra, ~, n_tars_max_rtra, max_rtra_inx] = robot_tra_cover_fun(r_pos, tar_pos);

% find all the non-overlapping maximal cliques
[nonovlap_cliqs_G, num_of_cliqs_G, cliq_num_G] = nonoverlap_cliqs_fun(r_pos);

% calcuate the non-overlapping cliques of G3
nonovlap_cliqs_G3 = {}; 
% the # of robots in each cliq of G3
each_cliq_num = []; 
% select out the first N_atk in each clique of G
r_tra_o_local = [];  
% for each clique do a local oblivous algorithm first
for i = 1 : length(nonovlap_cliqs_G)
    %store the assignment for the robots in each clique
    if length(nonovlap_cliqs_G{i}) <= N_atk
        r_tra_o = oblivious_fun(nonovlap_cliqs_G{i}, length(nonovlap_cliqs_G{i}),...\
            n_tars_max_rtra, max_rtra_inx); 
    else 
        r_tra_o = oblivious_fun(nonovlap_cliqs_G{i}, N_atk,...\
            n_tars_max_rtra, max_rtra_inx);        
    end
     %temp_cliq_G3
     temp_nonovlap_cliqs_G3 = setdiff(nonovlap_cliqs_G{i}, r_tra_o(:,1));
     if ~isempty(temp_nonovlap_cliqs_G3)
         nonovlap_cliqs_G3 = [nonovlap_cliqs_G3; temp_nonovlap_cliqs_G3];
         each_cliq_num = [each_cliq_num, length(temp_nonovlap_cliqs_G3)];
     end
     r_tra_o_local = [r_tra_o_local; r_tra_o];
end
% clique number of G3
cliq_num_G3 = 0;
% the # of cliques in G3
num_of_cliqs_G3 = 0; 
if ~isempty(nonovlap_cliqs_G3)
    cliq_num_G3 = max(each_cliq_num);
    num_of_cliqs_G3 = length(nonovlap_cliqs_G3);
end

% perform an centralized oblivious greedy algorithm in the collected local biggest
% alpha ones in central server
cen_cliq = r_tra_o_local(:,1); 
r_tra_og_cen = oblivious_greedy_fun(cen_cliq, tar_set_rtra, n_tars_max_rtra, max_rtra_inx); 

% pefrom a local greedy algorithm in each remaining clique of G3
r_tra_g_local = []; 
if ~isempty(nonovlap_cliqs_G3)
    % for each clique do a greedy algorithm
    for i = 1 : length(nonovlap_cliqs_G3)
        %store the assignment for the robots in each clique
        [r_tra_g_each]= greedy_fun(nonovlap_cliqs_G3{i}, tar_set_rtra); 
        r_tra_g_local = [r_tra_g_local; r_tra_g_each]; 
    end
else
    r_tra_g_local = [];
end
% calculate the tra_asign
r_tra_hier_obli_gre = [r_tra_og_cen; r_tra_g_local]; 
% calculate the targets tracked
[n_hier_obli_gre] = n_tra_cover(tar_set_rtra, r_tra_hier_obli_gre); 
% after getting the trajectories for all the robots, tested by worst attack
[n_hier_obli_gre_atk]= worst_attack(tar_set_rtra, r_tra_hier_obli_gre);
% worst attack rate
 worst_att_rate_cen_og = n_hier_obli_gre_atk/n_hier_obli_gre;
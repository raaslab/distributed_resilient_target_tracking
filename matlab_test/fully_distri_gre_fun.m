function [n_gre_after_atk, atk_rate_gre, com_fd_g, t_fd_g] ...
    = fully_distri_gre_fun(tar_set_rtra, nonovlap_cliqs_G, com_cliq_form, t_cliq_form)
            
    tic;
    % give the trajectory assignments for the robots in each clique
    r_tra_gre = []; 
    % the communication in each clique 
    com_in_cliq = zeros(1, length(nonovlap_cliqs_G)); 
    % the number of computational evaluations for each clique
    eva_in_cliq = zeros(1, length(nonovlap_cliqs_G));
    % for each clique do an oblivous-greedy algorithm
    for i = 1 : length(nonovlap_cliqs_G)
        %store the assignment for the robots in each clique
        [r_tra_gre_c, eva_g]= greedy_fun(nonovlap_cliqs_G{i}, [ ], tar_set_rtra); 
        r_tra_gre = [r_tra_gre; r_tra_gre_c]; 
        
        eva_in_cliq(i) = eva_g;
        if length(nonovlap_cliqs_G{i}) > 1
            com_in_cliq(i) = nchoosek(length(nonovlap_cliqs_G{i}),2);
        else
            com_in_cliq(i) = 0;
        end
    end
    % store the running time of the algorithm
    t_fd_g = toc/length(nonovlap_cliqs_G) + t_cliq_form;
    % calculate the communication, contains two parts, one from cliq formulation
    % one from the fully distribtued algorithm. 
    com_fd_g =  max(com_in_cliq) + com_cliq_form; 
    % calculate the number of evaluations of the algorithm, choose the max
    % among the cliques
    eav_fd_g = max(eva_in_cliq);
    % calculate the targets tracked
    [n_resi] = n_tra_cover(tar_set_rtra, r_tra_gre); 
    % after getting the trajectories for all the robots, tested by worst attack
    [n_gre_after_atk]= worst_attack(tar_set_rtra, r_tra_gre);
    % worst attack rate
    atk_rate_gre = n_gre_after_atk/n_resi; 
end
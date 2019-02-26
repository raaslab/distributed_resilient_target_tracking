function [n_resi_after_atk, worst_atk_rate_resi, ...
    com_fd, t_fd] = ...
        fully_distri_resi_fun(tar_set_rtra, ...
        nonovlap_cliqs_G, n_tars_max_rtra, max_rtra_inx)
    tic;
    % give the trajectory assignments for the robots in each clique
    r_tra_resi = []; 
    % the communication in each clique 
    com_in_cliq = zeros(1, length(nonovlap_cliqs_G)); 
    % the number of computational evaluations for each clique
    eva_in_cliq = zeros(1, length(nonovlap_cliqs_G));
    % for each clique do an oblivous-greedy algorithm
    for i = 1 : length(nonovlap_cliqs_G)
        %store the assignment for the robots in each clique
        [r_tra_resi_c, ~, eva_og]= oblivious_greedy_fun(nonovlap_cliqs_G{i}, tar_set_rtra,...
                                                n_tars_max_rtra, max_rtra_inx); 
        r_tra_resi = [r_tra_resi; r_tra_resi_c]; 
        eva_in_cliq(i) = eva_og;
        if length(nonovlap_cliqs_G{i}) > 1
            com_in_cliq(i) = nchoosek(length(nonovlap_cliqs_G{i}),2);
        else
            com_in_cliq(i) = 0;
        end
    end
    % store the running time of the algorithm
    t_fd = toc/length(nonovlap_cliqs_G);
    % calculate the communication, contains two parts, one from cliq formulation
    % one from the fully distribtued algorithm. 
    com_fd =  max(com_in_cliq); 
    % calculate the number of evaluations of the algorithm, choose the max
    % among the cliques
    eav_fd = max(eva_in_cliq);
    % calculate the targets tracked
    [n_resi] = n_tra_cover(tar_set_rtra, r_tra_resi); 
    % after getting the trajectories for all the robots, tested by worst attack
    [n_resi_after_atk]= worst_attack(tar_set_rtra, r_tra_resi);
    % worst attack rate
     worst_atk_rate_resi = n_resi_after_atk/n_resi; 
end
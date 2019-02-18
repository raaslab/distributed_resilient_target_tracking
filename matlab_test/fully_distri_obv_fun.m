function [n_obv_after_atk, atk_rate_obv, com_dis_o, t_dis_o] ...
    = fully_distri_obv_fun(tar_set_rtra, nonovlap_cliqs_G, ...
    n_tars_max_rtra, max_rtra_inx)
         
    % give the trajectory assignments for the robots in each clique
    r_tra_obv = []; 
    % the communication in each clique 
    com_in_cliq = zeros(1, length(nonovlap_cliqs_G)); 
    
    % the number of computational evaluations for each clique
    eva_in_cliq = zeros(1, length(nonovlap_cliqs_G));
    
    % for each clique do an oblivous algorithm
    for i = 1 : length(nonovlap_cliqs_G)
        %store the assignment for the robots in each clique
        
        [r_tra_obv_c, eva_o]= oblivious_fun(nonovlap_cliqs_G{i},length(nonovlap_cliqs_G{i}), ...
            n_tars_max_rtra, max_rtra_inx);  
            
        r_tra_obv = [r_tra_obv; r_tra_obv_c];
        eva_in_cliq(i) = eva_o;
        
        if length(nonovlap_cliqs_G{i}) > 1
            com_in_cliq(i) = nchoosek(length(nonovlap_cliqs_G{i}),2);
        else
            com_in_cliq(i) = 0;
        end
    end
    % store the running time of the algorithm
    t_dis_o = toc/length(nonovlap_cliqs_G);
    % calculate the communication, contains two parts, one from cliq formulation
    % one from the fully distribtued algorithm. 
    com_dis_o = sum(com_in_cliq); 
    % calculate the number of evaluations of the algorithm, choose the max
    % among the cliques
    eav_dis_o = max(eva_in_cliq);
    % calculate the targets tracked
    [n_resi] = n_tra_cover(tar_set_rtra, r_tra_obv); 
    % after getting the trajectories for all the robots, tested by worst attack
    [n_obv_after_atk]= worst_attack(tar_set_rtra, r_tra_obv);
    % worst attack rate
     atk_rate_obv = n_obv_after_atk/n_resi; 
end
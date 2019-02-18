function [n_hier_obli_atk, worst_att_rate_cen_o, num_of_cliqs_G2, cliq_num_G2,...
        com_cen_o, eva_cen_o] = ...
        hierarch_cen_obli_fun(r_pos, tar_set_rtra, n_tars_max_rtra, max_rtra_inx)

    global N_atk  
    % find all the non-overlapping maximal cliques
    [nonovlap_cliqs_G, ~, ~, com_cliq_form] = nonoverlap_cliqs_fun(r_pos);

    % select out the first N_atk in each clique of G
    r_tra_o_local = [];
    
    % the communication in each clique 
    com_in_cliq = zeros(1, length(nonovlap_cliqs_G));
    
    % the number of computational evaluations for each clique by a local
    % oblivious algorithm
    eva_o_in_cliq = zeros(1, length(nonovlap_cliqs_G));
    
    % for each clique do a local oblivous algorithm first
    for i = 1 : length(nonovlap_cliqs_G)
        %store the assignment for the robots in each clique
        if length(nonovlap_cliqs_G{i}) <= N_atk
            [r_tra_o_in, eva_o] = oblivious_fun(nonovlap_cliqs_G{i}, ...
                length(nonovlap_cliqs_G{i}), n_tars_max_rtra, max_rtra_inx); 
        else 
            [r_tra_o_in, eva_o] = oblivious_fun(nonovlap_cliqs_G{i}, N_atk,...\
                n_tars_max_rtra, max_rtra_inx);        
        end
        r_tra_o_local = [r_tra_o_local; r_tra_o_in];
        eva_o_in_cliq(i) = eva_o;
        
        if length(nonovlap_cliqs_G{i}) > 1
            com_in_cliq(i) = nchoosek(length(nonovlap_cliqs_G{i}),2);
        else
            com_in_cliq(i) = 0;
        end
    end
    % the evaluations of local oblivious
    eva_o_local = max(eva_o_in_cliq);  
    
    % the communication from cliqs to central server,
    com_to_cen = length(nonovlap_cliqs_G);
    
    % the communication within the central server 
    % should be 0.
    % because the central server alreay receive the info from all cliques
    com_in_cen = 0;
    
    % alpha ones in central server
    cen_cliq = r_tra_o_local(:,1); 
    
    % choose out the first alpha N_atk biggest ones by an oblivious algorithm
    [r_tra_o, eva_o_central] = oblivious_fun(cen_cliq, N_atk, n_tars_max_rtra, max_rtra_inx);
    
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
    
    % communication from the central server to each cliq
    com_to_cliq = num_of_cliqs_G2;

    % give the trajectory assignments for the robots in each clique of G2
    r_tra_g = []; 
    % the number of computational evaluations for each clique by a local
    % greedy algorithm in G2
    eva_g_in_cliq = zeros(1, length(nonovlap_cliqs_G2));
    % for each clique do an oblivous-greedy algorithm
    for i = 1 : length(nonovlap_cliqs_G2)
        %store the assignment for the robots in each clique
        [r_tra_each_g, eva_g]= greedy_fun(nonovlap_cliqs_G2{i}, [ ], tar_set_rtra); 
        r_tra_g = [r_tra_g; r_tra_each_g]; 
        eva_g_in_cliq(i) = eva_g; 
    end
    % the evaluations of local greedy
    eva_g_local = max(eva_g_in_cliq);
    
    % calculate the communication, contains two parts, one from cliq formulation
    % one from the fully distribtued algorithm: com within each cliq, com to central server  
    % com within the central server, and com to cliqs
    com_cen_o = com_cliq_form + sum(com_in_cliq) + com_to_cen + com_in_cen + com_to_cliq;
    % the total evaluations
    eva_cen_o = eva_o_local + eva_o_central + eva_g_local; 
    % calculate the tra_asign
    r_tra_hier_obli = [r_tra_o; r_tra_g]; 
    % calculate the targets tracked
    [n_hier_obli] = n_tra_cover(tar_set_rtra, r_tra_hier_obli); 
    % after getting the trajectories for all the robots, tested by worst attack
    [n_hier_obli_atk] = worst_attack(tar_set_rtra, r_tra_hier_obli);
    % worst attack rate
    worst_att_rate_cen_o = n_hier_obli_atk/n_hier_obli;
end
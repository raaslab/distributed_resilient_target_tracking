function  [n_hier_obli_gre_atk, worst_atk_rate_cen_og, num_of_cliqs_G3, cliq_num_G3, ...
    com_cen_og, eva_cen_og] = ...
    hierarch_cen_obli_gre_fun(r_pos, tar_set_rtra, n_tars_max_rtra, max_rtra_inx)
    

    global N_atk
    % find all the non-overlapping maximal cliques
    [nonovlap_cliqs_G, ~, ~, com_cliq_form] = nonoverlap_cliqs_fun(r_pos);

    % calcuate the non-overlapping cliques of G3
    nonovlap_cliqs_G3 = {}; 
    % the # of robots in each cliq of G3
    each_cliq_num = []; 
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
            [r_tra_o, eva_o] = oblivious_fun(nonovlap_cliqs_G{i}, length(nonovlap_cliqs_G{i}),...\
                n_tars_max_rtra, max_rtra_inx); 
        else 
            [r_tra_o, eva_o] = oblivious_fun(nonovlap_cliqs_G{i}, N_atk,...\
                n_tars_max_rtra, max_rtra_inx);        
        end
         %temp_cliq_G3
         temp_nonovlap_cliqs_G3 = setdiff(nonovlap_cliqs_G{i}, r_tra_o(:,1));
         if ~isempty(temp_nonovlap_cliqs_G3)
             nonovlap_cliqs_G3 = [nonovlap_cliqs_G3; temp_nonovlap_cliqs_G3];
             each_cliq_num = [each_cliq_num, length(temp_nonovlap_cliqs_G3)];
         end
         r_tra_o_local = [r_tra_o_local; r_tra_o];
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

    
    % clique number of G3
    cliq_num_G3 = 0;
    % the # of cliques in G3
    num_of_cliqs_G3 = 0; 
    if ~isempty(nonovlap_cliqs_G3)
        cliq_num_G3 = max(each_cliq_num);
        num_of_cliqs_G3 = length(nonovlap_cliqs_G3);
    end

    % the communication within the central server 
    % should be 0.
    % because the central server alreay receive the info from all cliques
    com_in_cen = 0;
    
    % perform an centralized oblivious greedy algorithm in the collected local biggest
    % alpha ones in central server
    cen_cliq = r_tra_o_local(:,1); 
     
    [r_tra_og_cen, r_tra_g_cen, eva_og_central]= ...
        oblivious_greedy_fun(cen_cliq, tar_set_rtra, n_tars_max_rtra, max_rtra_inx); 
    
    % communication from the central server to each cliq
    com_to_cliq = num_of_cliqs_G3;

    % pefrom a local greedy algorithm in each remaining clique of G3...
    % wrong!!!!... greedy based on previously!!!...
    r_tra_g_local = []; 
    if ~isempty(nonovlap_cliqs_G3)
        % the number of computational evaluations for each clique by a local
        % greedy algorithm in G3
        eva_g_in_cliq = zeros(1, length(nonovlap_cliqs_G3));
        % for each clique do a greedy algorithm
        for i = 1 : length(nonovlap_cliqs_G3)
            %store the assignment for the robots in each clique,
            %note this greedy should be based on the grredy in the
            %central server
            [r_tra_g_each, eva_g]= ...
                greedy_fun(nonovlap_cliqs_G3{i}, r_tra_g_cen, tar_set_rtra); 
            
            r_tra_g_local = [r_tra_g_local; r_tra_g_each]; 
            eva_g_in_cliq(i) = eva_g;
        end
        eva_g_local = max(eva_g_in_cliq);
    else
        r_tra_g_local = [];
        eva_g_local = 0;
    end
    % calculate the communication, contains two parts, one from cliq formulation
    % one from the fully distribtued algorithm: com within each cliq, com to central server  
    % and com within the central server
    com_cen_og = com_cliq_form + sum(com_in_cliq) + com_to_cen + com_in_cen + com_to_cliq;
    % the total evaluations
    eva_cen_og = eva_o_local + eva_og_central + eva_g_local;
    % calculate the tra_asign
    r_tra_hier_obli_gre = [r_tra_og_cen; r_tra_g_local]; 
    % calculate the targets tracked
    [n_hier_obli_gre] = n_tra_cover(tar_set_rtra, r_tra_hier_obli_gre); 
    % after getting the trajectories for all the robots, tested by worst attack
    [n_hier_obli_gre_atk]= worst_attack(tar_set_rtra, r_tra_hier_obli_gre);
    % worst attack rate
    worst_atk_rate_cen_og = n_hier_obli_gre_atk/n_hier_obli_gre;
end
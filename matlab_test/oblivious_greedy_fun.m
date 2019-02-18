% function: assign a trajectory to each robot by the ovlivious-greedy algorithm
function [r_tra_og, r_tra_g, eva_og] = ...
    oblivious_greedy_fun(cliq, tar_set_rtra, n_tars_max_rtra, max_rtra_inx)
    global N_atk
    % if the number of robots in the clique is less than the attack number,
    % atk, do oblivious for this cliq
    if length(cliq) <= N_atk
        [r_tra_og, eva_og] = oblivious_fun(cliq, length(cliq), n_tars_max_rtra, max_rtra_inx);
        r_tra_g = [];
    else % first do oblivious
        [r_tra_o, eva_o] = oblivious_fun(cliq, N_atk, n_tars_max_rtra, max_rtra_inx);
        % compute the remaining robots from r_tra_o
        cliq_remain = setdiff(cliq, r_tra_o(:,1)); 
        % then do greedy
        [r_tra_g, eva_g] = greedy_fun(cliq_remain, [ ], tar_set_rtra);
        % oblivious + greedy
        r_tra_og = [r_tra_o; r_tra_g];
        % the number of evaluations
        eva_og = eva_o + eva_g; 
    end
end
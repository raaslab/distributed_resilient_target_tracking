% function: assign a trajectory to each robot by the ovlivious-greedy algorithm
function [r_tra_og, r_tra_g] = oblivious_greedy_fun(cliq, target_cover, n_id_maxtra)
    
    global N_fail_uavs
    
    % if the number of robots in the clique is less than the attack number,
    % atk, do oblivious for this cliq
    if length(cliq) <= N_fail_uavs
        [r_tra_og] = oblivious_fun(cliq, length(cliq), n_id_maxtra);
        r_tra_g = [];
    else % first do oblivious
        [r_tra_o] = oblivious_fun(cliq, N_fail_uavs, n_id_maxtra);
        % compute the remaining robots from r_tra_o
        cliq_remain = setdiff(cliq, r_tra_o(:,1)); 
        % then do greedy
        [r_tra_g] = greedy_fun(cliq_remain, [ ], target_cover);
        % oblivious + greedy
        r_tra_og = [r_tra_o; r_tra_g]; 
    end
end
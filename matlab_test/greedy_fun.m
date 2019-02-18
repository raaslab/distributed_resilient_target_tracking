% function: assign trajectory to robot by a greedy algorithm
% select the trajectory by greedy. 
function [r_tra_g, eva_g] = greedy_fun(r_set, r_tra, tar_set_rtra)
    global N_tra
    r_tra_g = zeros(length(r_set), 2); 
    tar_gre_set = cell(length(r_set)+1, 1);  
    
    % store the number of evaluations of the greedy algorithm
    eva_g = (length(r_set) * N_tra) * (length(r_set) * N_tra + 1)/2;
    
    % compute the tar_gre_set{1}
    if ~isempty(r_tra) % if isempty is not empty. 
        for r_idx = 1 : length(r_tra(:,1))
            tar_gre_set{1} = union(tar_gre_set{1}, tar_set_rtra{r_tra(r_idx,1), r_tra(r_idx,2)});
        end
    end
    
    for k = 1 : length(r_set) % length(set) rounds
        gre_base = 0;
        for i = 1 : length(r_set) % check the robots
            for j = 1 : N_tra % check the trajectories
                margin_gain = length(union(tar_set_rtra{r_set(i),j}, tar_gre_set{k})) ...
                             -length(tar_gre_set{k});
                if margin_gain >= gre_base
                    gre_base = margin_gain;
                    r_tra_g(k, :) = [r_set(i), j];
                end
            end
        end
        % after a round is done, r_set should be reset, since one robot is charged 
        r_set = setdiff(r_set, r_tra_g(k,1)); 
        % greedy_base set should be update 
        tar_gre_set{k+1} = union(tar_gre_set{k}, tar_set_rtra{r_tra_g(k,1),r_tra_g(k,2)});
    end

end
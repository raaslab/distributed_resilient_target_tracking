% function: assign trajectory to robot by a greedy algorithm
% select the trajectory by greedy. 
function [r_tra_g] = greedy_fun(r_set, r_tra, target_cover)
    global N_dir_uav
    global uavs_pos fly_length

    r_tra_g = zeros(length(r_set), 2); 
    tar_gre_set = cell(length(r_set)+1, 1);  
       
    % compute the tar_gre_set{1}
    if ~isempty(r_tra) % if isempty is not empty. 
        for r_idx = 1 : length(r_tra(:,1))
            tar_gre_set{1} = union(tar_gre_set{1}, target_cover{r_tra(r_idx,1), r_tra(r_idx,2)});
        end
    end
    
    for k = 1 : length(r_set) % length(set) rounds
        gre_base = 0;
        for i = 1 : length(r_set) % check the robots
            margin_gain = zeros(N_dir_uav, 1); 
            dis_origin = zeros(N_dir_uav, 1); 
            for j = 1 : N_dir_uav % check the trajectories
                margin_gain(j) = length(union(target_cover{r_set(i),j}, tar_gre_set{k})) ...
                             -length(tar_gre_set{k});
                if j == 1
                    dis_origin(j) = norm([uavs_pos(i,1), uavs_pos(i,2) + fly_length]);
                elseif j == 2
                    dis_origin(j) = norm([uavs_pos(i,1), uavs_pos(i,2)- fly_length]);
                elseif j ==3 
                    dis_origin(j) = norm([uavs_pos(i,1) + fly_length, uavs_pos(i,2)]);
                else 
                    dis_origin(j) = norm([uavs_pos(i,1) - fly_length, uavs_pos(i,2)]);
                end
%                 if margin_gain >= gre_base
%                     gre_base = margin_gain;
%                     r_tra_g(k, :) = [r_set(i), j];
%                 end
            end
            [margin_gain_max, gain_max_inx] = max(margin_gain);
            if margin_gain_max >= gre_base
                gre_base = margin_gain_max;
                r_tra_g(k, :) = [r_set(i), gain_max_inx]; 
                % all directions are equal
                if all(margin_gain == margin_gain(1))
                    [~, dis_min_inx] = min(dis_origin); 
                    r_tra_g(k, :) = [r_set(i), dis_min_inx]; 
                end
            end
        end
        % after a round is done, r_set should be reset, since one robot is charged 
        r_set = setdiff(r_set, r_tra_g(k,1)); 
        % greedy_base set should be update 
        tar_gre_set{k+1} = union(tar_gre_set{k}, target_cover{r_tra_g(k,1),r_tra_g(k,2)});
    end
end
% function: compute the number of targets remained 
% after the worst-case attack
function [n_after_att]= worst_attack(tar_set_rtra, r_tra)
    
        global Nr N_tra N_atk
        r_tra_idx = zeros(1,Nr); % give each (robot,tra) pair an index
        for i = 1 : Nr
            r_tra_idx(i) = (r_tra(i,1)-1) * N_tra + r_tra(i,2); 
        end
        % choose N_atk out of r_tra_index
        nchoose_katk_index = nchoosek(r_tra_idx, N_atk); 
        [row_nchokatk, col_nchokatk] = size(nchoose_katk_index); 
         remain_after_kfail_index = zeros(row_nchokatk, Nr-col_nchokatk); 
         remain_after_kfail = cell(1, row_nchokatk); 
         num_remain_after_kfail = zeros(1,row_nchokatk);

        for i = 1:row_nchokatk
            remain_after_kfail_index(i,:) = setdiff(r_tra_idx, nchoose_katk_index(i,:));
            [~, col_remian] = size(remain_after_kfail_index(i,:));

            temp_remain_after_kfail = cell(1, col_remian+1);
            for j = 1: col_remian
            % corresping to whcih robot and which traj
               r_remain_index =fix((remain_after_kfail_index(i,j)-1)/N_tra)+1; 
               tra_remain_index =mod(remain_after_kfail_index(i,j)-1, N_tra)+1;
               temp_remain_after_kfail{j+1} = union(tar_set_rtra{r_remain_index,tra_remain_index},...
                   temp_remain_after_kfail{j});
            end
             remain_after_kfail{i} = temp_remain_after_kfail{col_remian+1};
             num_remain_after_kfail(i) = length(remain_after_kfail{i}); 
        end
        n_after_att = min(num_remain_after_kfail);
end
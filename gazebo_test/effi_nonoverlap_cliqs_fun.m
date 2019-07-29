% function: generate non-overlapping cliques among robots in an efficient way
function [nonoverlap_cliqs, num_of_cliqs, cliq_num, cliq_id] = ...
    effi_nonoverlap_cliqs_fun(r_pos)
    
    global N_uavs nei_range
    global com_cliq_form t_cliq_form
   
    % store robot i and its neighbors
    r_neighbor = cell(N_uavs,1);    
    % 1st communication: recongize neighbors
    com_nei = zeros(N_uavs, 1);
    
    for i = 1 : N_uavs
        r_neighbor{i} = i; 
        for j = 1 : N_uavs
            % search for neighbors 
            if j~=i && norm(r_pos(j,:)-r_pos(i,:)) <= nei_range
                r_neighbor{i} = [r_neighbor{i}, j];
            end
        end
        com_nei(i) = length(r_neighbor{i})-1;
    end
    % after this for-loop, we have each robot's neighbors
    
    % start computation
    tic; 
    % store a clqiue for each r
    r_in_cliq = cell(N_uavs,1); 

    for i = 1 : N_uavs % for all the robots

        % 2nd communication with neighbors
        % find its neighbors and their degree
        r_nei_degree = zeros(length(r_neighbor{i})-1, 2);

        % if r_neighbor only has robot i itself
        if length(r_neighbor{i}) == 1 
             r_in_cliq{i} = r_neighbor{i};
        else % r_neighbor has other robot except robot i
            for n = 1 : length(r_neighbor{i})-1 % we don't consider robot i itself
                r_nei_degree(n,:) = [r_neighbor{i}(n+1), length(r_neighbor{r_neighbor{i}(n+1)})];
            end
            % find its neighbors with maximal degree
            [~, max_inx] = max(r_nei_degree(:,2));
            % maximal_cliqe for each robot
            r_in_cliq{i} = intersect(r_neighbor{i}, r_neighbor{r_nei_degree(max_inx,1)});
        end

        % after getting an unique clique, check if this robot is already
        % selected in previous cliques. Then decide to join which one
        %3rd communication with neighbors, 
        for nei = 1 : i
            % calculate the intersection
            insect = intersect(r_in_cliq{nei}, r_in_cliq{i});
            % if they have intersection
            if ~isempty(insect) && nei ~= i
                % if r's clique is larger than its neighbor's, neighbor's cliq loses intesec
                if length(r_in_cliq{i}) > length(r_in_cliq{nei}) 
                    r_in_cliq{nei} = setdiff(r_in_cliq{nei}, insect);
                else % else, r's clique is smaller than its neighbor's, it loses intesect
                    r_in_cliq{i} = setdiff(r_in_cliq{i}, insect);
                end
            end
        end
    end
    % Overall 3 communications with neighbors, is meaningless...
    com_cliq_form = max(com_nei)*3/2;
    % running time
    t_cliq_form = toc/N_uavs;
    % compute and store non-overlapping cliques
    nonoverlap_cliqs = { }; 
    each_cliq_num = []; 
    for i = 1 : length(r_in_cliq)
        flag_repeat_clique = ones(1,length(nonoverlap_cliqs)); 
        for j = 1 : length(nonoverlap_cliqs)
            if ~isempty(setdiff(r_in_cliq{i}, nonoverlap_cliqs{j}))
                flag_repeat_clique(j) = 0;
            end
        end
        if sum(flag_repeat_clique) == 0 && ~isempty(r_in_cliq{i})
            nonoverlap_cliqs = [nonoverlap_cliqs; r_in_cliq{i}];
            each_cliq_num = [each_cliq_num, length(r_in_cliq{i})];
        end
    end
    [cliq_num, ~] = max(each_cliq_num); 
    num_of_cliqs = length(nonoverlap_cliqs);
    
    % return cliq id for each uav
    cliq_id = zeros(N_uavs, 1);
    for i = 1 : N_uavs
        for j = 1 : length(nonoverlap_cliqs)
            if ismember(i, nonoverlap_cliqs{j})
                cliq_id(i) = j; 
            end
        end
    end
end
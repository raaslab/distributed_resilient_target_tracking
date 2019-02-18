% compute non-overlapping cliques on a graph G 
% we only give a FEASIBLE distribtued solution

% the number of robots, 
N = 6;
% give the postions of all robots
r_pos = zeros(N,2);
r_pos(:, 1) = 100 * rand(N, 1);
r_pos(:, 2) = 100 * rand(N, 1);

% neighbor range, within the distance
nei_range = 50;

% store robot i and its neighbors
r_nei_inx = cell(N,1);

for i = 1 : N
    r_nei_inx{i} = i; 
    for j = 1 : N
        % search for neighbors 
        if j~=i && norm(r_pos(j,:)-r_pos(i,:))<=nei_range
            r_nei_inx{i} = [r_nei_inx{i}, j];
        end
    end
end
%r_nei_inx = {[1 3 6];[2 6];[3 1 4 6];[4 3 5];[5 4];[6 1 2 3]};
% neighbor's neighbors are used to compute the clqiue by intersection
% store the clqiues for each r
r_in_clique = cell(N,1); 
for i = 1 : N 
    % store the maximal clqiues for each robot, since a robot may have more than one 
    temp_cliques_r = {};
    % the # of robot i and its neighbors 
    for n_ri_nei = length(r_nei_inx{i}) : -1 : 1
        % the possible cases of intersection
        in_sect = nchoosek(r_nei_inx{i}, n_ri_nei);
        % the numebr of cases of itersection
        for t = 1 : length(in_sect(:,1))
            % define a set to store the intersections
            temp_insect = (1:N); 
            for k = 1 : length(in_sect(t,:))
                temp_insect = intersect(temp_insect, r_nei_inx{in_sect(t,k)});
            end
            
            % check if it is a maximal clique
            if length(temp_insect) ==  n_ri_nei
                %check if temp_insect has overlapping with cliques_r
                %if no overlap with elements in cliques_r
                flag_overlap = ones(1,i);
                cliq_pend_inx = 0; 
                for p = 1 : i
                    %no overlap
                    if isempty(intersect(temp_insect, r_in_clique{p}))
                        flag_overlap(p) = 0;
                    else 
                        %overlap but equal
                        if isempty(setdiff(temp_insect, r_in_clique{p}))
                            flag_overlap(p) = 0;
                        %overlap, but current is larger, grab the intersect
                        %back
                        elseif length(temp_insect) > length(r_in_clique{p})
                            % a clqiue is pending to loose node
                            cliq_pend_inx = p; 
                            %r_in_clique{p} = setdiff(r_in_clique{p},temp_insect); 
                            flag_overlap(p) = 0;
                        end
                    end
                end
                % no overlap or equal fulfils all elements in cliques_r
                if sum(flag_overlap) == 0 % all previous clqiues agree
                    % if all agreed, new clique is born,
                    temp_cliques_r = [temp_cliques_r, temp_insect];
                    % some old smaller clqiue loses node
                    if cliq_pend_inx ~= 0
                        r_in_clique{cliq_pend_inx} = setdiff(r_in_clique{cliq_pend_inx},...
                            temp_insect);
                    end
                end

            end
        end
        if ~isempty(temp_cliques_r)
                if length(temp_cliques_r)==1
                    r_in_clique{i} = temp_cliques_r{1}; 
                else
                    % we need to decide which one by comparing the
                    % number of robots in these cliques, choose the
                    % smallest one
                    
                    % calculate the number of all the neigbors of the
                    % members in each clique
                    n_cliq_nei = zeros(1,length(temp_cliques_r));
                    for m = 1 : length(temp_cliques_r)
                        each_cliq_nei = [];
                     for n = 1 : length(temp_cliques_r{m})
                         each_cliq_nei = union(each_cliq_nei, r_nei_inx{temp_cliques_r{m}(n)}); 
                     end 
                         n_cliq_nei(m) = length(each_cliq_nei);
                    end
                    % choose the clqiue which has the minimum each_cliq_nei 
                    [~,min_inx] = min(n_cliq_nei);
                    r_in_clique{i} = temp_cliques_r{min_inx};
                end
            break;
        end
    end
end
% double check if r_in_clique{i} = {}, give i to it, a clique with single
% node
for i = 1 : N
    if isempty(r_in_clique{i})
        r_in_clique{i} = i; 
    end
end
% calculate the non overlap cliques
% store all non-overlapping clqiues
nonoverlap_cliques = { }; 
for i = 1 : length(r_in_clique)
    flag_repeat_clique = ones(1,length(nonoverlap_cliques)); 
    for j = 1 : length(nonoverlap_cliques)
        if ~isempty(setdiff(r_in_clique{i}, nonoverlap_cliques{j}))
            flag_repeat_clique(j) = 0;
        end
    end
    if sum(flag_repeat_clique) == 0
        nonoverlap_cliques = [nonoverlap_cliques; r_in_clique{i}];
    end
end

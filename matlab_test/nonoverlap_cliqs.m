% % compute non-overlapping cliques on a graph G 
% % we only give a FEASIBLE distribtued solution
% 
% % the number of robots, 
% Nr = 6;
% % give the postions of all robots
% r_pos = zeros(Nr,2);
% r_pos(:, 1) = 100 * rand(Nr, 1);
% r_pos(:, 2) = 100 * rand(Nr, 1);

%%
% neighbor range, within the distance
Nr = 10;
nei_range = 50;

r_pos = [43.736097320218107  93.410446177255096
  83.629640732425003  47.750709597826159
  68.157660161731499  60.347793864945245
  30.095046586899610  55.703523793240784
  23.312920245258397  11.032265227536008
  21.337097096324232  50.757009969983891
  62.192977838792032  92.409091768479215
  23.420395607668397  22.934738136958032
   6.571907389024101  26.942398145015090
  46.609821610767099  13.263947712151779];

% store robot i and its neighbors
r_neighbor = cell(Nr,1);

for i = 1 : Nr
    r_neighbor{i} = i; 
    for j = 1 : Nr
        % search for neighbors 
        if j~=i && norm(r_pos(j,:)-r_pos(i,:)) <= nei_range
            r_neighbor{i} = [r_neighbor{i}, j];
        end
    end
end
%r_nei_inx = {[1 3 6];[2 6];[3 1 4 6];[4 3 5];[5 4];[6 1 2 3]};
% neighbor's neighbors are used to compute the clqiue by intersection
% store the clqiues for each r
r_in_clique = cell(Nr,1); 
for i = 1 : Nr 
    % store the maximal clqiues for each robot, since a robot may have more than one 
    temp_cliques_r = {};
    % the # of robot i and its neighbors 
    for n_ri_nei = length(r_neighbor{i}) : -1 : 1
        % the possible cases of intersection
        in_sect = nchoosek(r_neighbor{i}, n_ri_nei);
        % the numebr of cases of itersection
        for t = 1 : length(in_sect(:,1))
            % define a set to store the intersections
            temp_insect = (1 : Nr); 
            for k = 1 : length(in_sect(t,:))
                temp_insect = intersect(temp_insect, r_neighbor{in_sect(t,k)});
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
                         each_cliq_nei = union(each_cliq_nei, r_neighbor{temp_cliques_r{m}(n)}); 
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
for i = 1 : Nr
    i_in_cliq_flag = ones(1, Nr);
    for j = 1 : Nr
        if ~ismember(i, r_in_clique{j})
            i_in_cliq_flag(j)=0;
        end
    end
    if isempty(r_in_clique{i}) && sum(i_in_cliq_flag) == 0
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
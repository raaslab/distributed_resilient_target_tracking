% compute non-overlapping cliques on a graph G 
% we only give a FEASIBLE distribtued solution

% the number of robots, 
Nr = 10;
% give the postions of all robots
r_pos = zeros(Nr,2);
r_pos(:, 1) = 100 * rand(Nr, 1);
r_pos(:, 2) = 100 * rand(Nr, 1);

nei_range = 70;

% plot positions
figure(1);
axis equal; box on; hold on;
xlim([0 100]);
ylim([0 100]);
plot(r_pos(:, 1), r_pos(:, 2), 'ko', 'MarkerSize', 10)

% % neighbor range, within the distance
% Nr = 10;
% nei_range = 50;
 
% r_pos = [43.736097320218107  93.410446177255096
%   83.629640732425003  47.750709597826159
%   68.157660161731499  60.347793864945245
%   30.095046586899610  55.703523793240784
%   23.312920245258397  11.032265227536008
%   21.337097096324232  50.757009969983891
%   62.192977838792032  92.409091768479215
%   23.420395607668397  22.934738136958032
%    6.571907389024101  26.942398145015090
%   46.609821610767099  13.263947712151779];

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
    % check if i is already selected
    i_in_cliq = 0;
    for p = 1 : i
        if ismember(i, r_in_clique{p})
            r_in_clique{i} = r_in_clique{p}; 
            i_in_cliq = 1;
            break;
        end
    end
    if i_in_cliq == 0
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
                    % if all agreed, new clique is born,
                    temp_cliques_r = [temp_cliques_r, temp_insect];
                    % some old smaller clqiue loses node
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
    %update the r_neighbor by breaking the connections with r_in_cliq;
    for k = 1 : length(r_in_clique{i})
        for l = 1 : Nr
            if r_in_clique{i}(k) == l
                r_neighbor{l} = [];
            elseif r_in_clique{i}(k) ~= l && ismember(r_in_clique{i}(k), r_neighbor{l})
                [~, idx] = ismember(r_in_clique{i}(k), r_neighbor{l}); 
                r_neighbor{l}(idx) = []; 
            end
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
% specify the number of colors
cmap = colormap(parula(length(nonoverlap_cliques)));

% plot the communication links
for i = 1 : length(nonoverlap_cliques)
    % plot the link within each clique
    if length(nonoverlap_cliques{i}) > 1
        pair = nchoosek(nonoverlap_cliques{i}, 2);
        % plot the link in each pair
        for j = 1 : length(pair(:,1))
            % pair(j,1) and pair(j,2) are two robots
            plot([r_pos(pair(j,1),1), r_pos(pair(j,2),1)], [r_pos(pair(j,1),2), r_pos(pair(j,2),2)],...
                'Color',cmap(i,:), 'LineWidth',1)
            hold on;
        end
    end
end
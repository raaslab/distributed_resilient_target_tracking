% compute non-overlapping cliques on a graph G 
% we only give a FEASIBLE distribtued solution

% the number of robots, 
Nr = 10;
% give the postions of all robots
r_pos = zeros(Nr,2);
r_pos(:, 1) = 100 * rand(Nr, 1);
r_pos(:, 2) = 100 * rand(Nr, 1);
nei_range = 50;

% %%
% % neighbor range, within the distance
% Nr = 10;
% nei_range = 50;
% 
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

% communciation to recongize neighbors 
com_nei = 0; 
for i = 1 : Nr
    r_neighbor{i} = i; 
    for j = 1 : Nr
        % search for neighbors 
        if j~=i && norm(r_pos(j,:) - r_pos(i,:)) <= nei_range
            r_neighbor{i} = [r_neighbor{i}, j];
        end
    end
    % -1 means the robot doesn't need to communicate with itself
    com_nei = com_nei + length(r_neighbor{i})-1;
end
% it is an undirected graph so that the communication is repeated once
com_nei = com_nei/2;
% 2nd communication: share neighbors' neighbors;
com_nei = com_nei*2;
%r_nei_inx = {[1 3 6];[2 6];[3 1 4 6];[4 3 5];[5 4];[6 1 2 3]};
% neighbor's neighbors are used to compute the clqiue by intersection
% store the clqiues for each r
r_in_clique = cell(Nr,1); 
% communication among neighbors' neighbors 
com_neinei = zeros(1, Nr);
for i = 1 : Nr %for all the robots
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
                    % if it is, store it, may have more than one
                    temp_cliques_r = [temp_cliques_r, temp_insect];             
                end
            end

            if ~isempty(temp_cliques_r) % if it has a maximial clique
                    if length(temp_cliques_r)==1 % and only one maximal clique
                        r_in_clique{i} = temp_cliques_r{1}; % use this one
                    else % if it has more than one maximal cliques
                        % we need to decide which one by comparing the # of
                        % neighbors for these cliques, choose the
                        % smallest one

                        % calculate the number of all the neigbors of the
                        % members in each clique
                        n_cliq_nei = zeros(1,length(temp_cliques_r));
                        for m = 1 : length(temp_cliques_r)
                            each_cliq_nei = [];
                         for n = 1 : length(temp_cliques_r{m})
                             each_cliq_nei = ...
                                 union(each_cliq_nei, r_neighbor{temp_cliques_r{m}(n)}); 
                         end 
                             n_cliq_nei(m) = length(each_cliq_nei);
                        end
                        
                        % choose the clqiue which has the minimum each_cliq_nei 
                        [~,min_inx] = min(n_cliq_nei);
                        r_in_clique{i} = temp_cliques_r{min_inx};
                    end
                    % after getting the unique clique, we need do one more
                    % thing, check if this robot is already selected in his
                    % neighbors cliques
                      for n_nei = 1 : i
                          % check if the current clique has intersection
                          % with neighbors' clqiues   
                              % if there exists intersect between current
                              % and the previous
                              insect = intersect(r_in_clique{n_nei}, r_in_clique{i});
                              if ~isempty(insect) && n_nei ~= i
                                  % if current clique is larger,
                                  % previous clique loses intersect,
                                  if length(r_in_clique{i}) > length(r_in_clique{n_nei}) 
                                      r_in_clique{n_nei} = setdiff(r_in_clique{n_nei}, insect);
                                      % if the robot change its clique, it
                                      % communicates with its neighbors. Notably, not
                                      % all robots need to do this.
                                      com_r_change_cliq = 0; 
                                      for c_ch = 1 : length(insect)
                                           com_r_change_cliq =  com_r_change_cliq + ...
                                               length(r_neighbor(insect(c_ch)))-1;
                                      end  
                                      com_neinei(i) = com_neinei(i) + com_r_change_cliq;
                                  else % current clique is smaller, or equal to, current loses
                                      r_in_clique{i} = setdiff(r_in_clique{i}, insect);
                                  end
                              end
                      end 
                      % communicate with neighbors to tell which
                      % cliques it is in for all the members in the select
                      % clique. 
                      com_r_in_cliq = 0;
                      for c_in = 1 : length(r_in_clique{i})
                          com_r_in_cliq = com_r_in_cliq + ...
                              length(r_neighbor(r_in_clique{i}(c_in)))-1;
                      end
                      com_neinei(i) = com_neinei(i) + com_r_in_cliq;                   
                break;
            end
        end    
end
%total communication is equal to the neighber and neihgbors's neighbor and
% tell neighbor its clique
com = com_nei + sum(com_neinei);
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
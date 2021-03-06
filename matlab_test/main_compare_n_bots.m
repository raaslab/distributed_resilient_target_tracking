% main file, compare different strategies with different strategies
% compare 5 stratgies: 
% fully distributed,
% hierarchcal_centralized_oblivious
% hierarchcal_centralized_oblivious_greedy
% centralized resilient
% centralized greedy
clear all;
global Nr N_tra Nt N_atk nei_range track_range

Nt = 60;

N_trials = 30;
  
% the # of robots, 
min_num_bots = 10; 
max_num_bots = 20; 

% the # of worst-case attacks
N_atk = 4; 

% the number of trajectories 
N_tra = 4; 

% neighbor range, within the distance, be carefully!!!
nei_range = 70;

% target tracking range
track_range = 10; 

% the range of robot position
pos_range = 100;

for Nr =  min_num_bots : max_num_bots
    robot_set_G = (1 : Nr);
    for i = 1 : N_trials
        r_pos = rand(Nr,2)*pos_range;
        tar_pos = rand(Nt,2)*pos_range;
        
     % find all the non-overlapping maximal cliques
    [nonovlap_cliqs_G, n_cliqs_G(i, Nr), cliq_num_G(i, Nr), ...
        com_cliq_form(i, Nr), t_cliq_form(i, Nr)] = effi_nonoverlap_cliqs_fun(r_pos);   
    
    % obtain a set of targets tracked by each trajectory
    [tar_set_rtra, ~, n_tars_max_rtra, max_rtra_inx] = robot_tra_cover_fun(r_pos, tar_pos);
    
    % centralized resilient algorithm
    [n_cen_resi(i, Nr), rate_cen_resi(i, Nr), com_cen_resi(i, Nr), t_cen_resi(i, Nr)] = ...
        cen_resi_fun(robot_set_G, tar_set_rtra, n_tars_max_rtra, max_rtra_inx);
    
    % centralized greedy algorithm
    [n_cen_gre(i, Nr), rate_cen_gre(i, Nr), com_cen_gre(i, Nr), t_cen_gre(i, Nr)] = ...
        cen_gre_fun(robot_set_G, tar_set_rtra); 
   
    % fully distribtued resilient
    [n_dis_resi(i, Nr), rate_dis_resi(i, Nr), com_dis_resi(i, Nr), t_dis_resi(i, Nr)] = ...
        distri_resi_fun(tar_set_rtra, ...
        nonovlap_cliqs_G, n_tars_max_rtra, max_rtra_inx);
    
%     % fully distribtued greedy
%     [n_dis_gre(i, Nr), rate_dis_gre(i, Nr), ...
%          com_dis_gre(i, Nr), t_dis_gre(i, Nr)] = ...
%         fully_distri_gre_fun(tar_set_rtra, nonovlap_cliqs_G, com_cliq_form, t_cliq_form);
    
%     % fully distributed oblivious
%     [n_dis_obv(i, Nr), rate_dis_obv(i, Nr), ...
%         com_dis_obv(i, Nr), eva_dis_obv(i, Nr)]...
%         = fully_distri_obv_fun(tar_set_rtra, nonovlap_cliqs_G, ...
%         n_tars_max_rtra, max_rtra_inx);   
    
%     % hierarchcal central server with an oblivious algorithm
%     [n_hier_cen_o(i, Nr), rate_hier_cen_o(i, Nr), ...
%         numofcliqs_G2(i, Nr), cliqnum_G2(i, Nr), com_cen_o(i, Nr), eva_cen_o(i, Nr)] = ...
%         hierarch_cen_obli_fun(r_pos, tar_set_rtra, n_tars_max_rtra, max_rtra_inx);
%     
%     % hierarchcal central server with an oblivious greedy algorithm
%     [n_hier_cen_og(i, Nr), rate_hier_cen_og(i, Nr), ...
%         numofcliqs_G3(i, Nr), cliqnum_G3(i, Nr), com_cen_og(i, Nr), eva_cen_og(i, Nr)] = ...
%         hierarch_cen_obli_gre_fun(r_pos, tar_set_rtra, n_tars_max_rtra, max_rtra_inx);   
 
    end
end
% plots 
  % compare the number of the remaining targets
  figure (1); hold on; grid on;
 
  shadedErrorBar(min_num_bots : max_num_bots, n_cen_resi(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', '--b','patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, n_cen_gre(:,min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':g','patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, n_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r','patchSaturation',0.33)
  
%   shadedErrorBar(min_num_bots : max_num_bots, n_dis_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops','--g','patchSaturation',0.33)  
  
%   shadedErrorBar(min_num_bots : max_num_bots, n_dis_obv(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33)   

%   shadedErrorBar(min_num_bots : max_num_bots, n_hier_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, n_hier_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
% '--m', '-y'

  title('comparison of the numebr of remaining targets tracked','fontsize',12)
%   legend('centralized resilient', 'centralized resilient, average', ...
%       'distributed resilient', 'distributed resilient, average',...
%       'distributed greedy', 'distributed greedy, average');
%     legend('', 'centralized resilient', '', 'centralized greedy', '', 'distributed resilient');
%   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');
  legend('centralized resilient', 'centralized resilient average', ...
      'centralized greedy', 'centralized greedy average',...
      'distributed resilient', 'distributed resilient average', 'FontSize',14);
  xlabel('Number of robots','fontsize', 14)
  ylabel('Number of targets tracked','fontsize', 14)  
  
% %   compare the worst-case attack rate
%     figure; hold on;grid on;
  
% %   shadedErrorBar(min_num_bots : max_num_bots, rate_cen_resi(:, min_num_bots : max_num_bots),...
% %       {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
%  
% %   shadedErrorBar(min_num_bots : max_num_bots, rate_cen_gre(:, min_num_bots : max_num_bots),...
% %       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33) 
%   
%   shadedErrorBar(min_num_bots : max_num_bots, rate_dis_resi(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)
%   
%   shadedErrorBar(min_num_bots : max_num_bots, rate_dis_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops','-.k', 'patchSaturation',0.33)  
%  
%   shadedErrorBar(min_num_bots : max_num_bots, rate_dis_obv(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b', 'patchSaturation',0.33)   
% 
% %   shadedErrorBar(min_num_bots : max_num_bots, rate_hier_cen_o(:,min_num_bots : max_num_bots),...
% %       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% % 
% %   shadedErrorBar(min_num_bots : max_num_bots, rate_hier_cen_og(:,min_num_bots : max_num_bots),...
% %       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
% %g 
%   title('comparison of removal rate','fontsize',12)
% %   legend('centralized resilient', 'centralized resilient, average', ...
% %       'distributed resilient', 'distributed resilient, average',...
% %       'distributed greedy', 'distributed greedy, average');
%   legend('distributed resilient', 'distributed resilient average',...
%       'distributed greedy', 'distributed greedy average', ...
%       'distributed oblivious', 'distributed oblivious average');  
% %  legend('', 'centralized resilient', '', 'centralized greedy', '', 'distributed resilient');
% %   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');
%   xlabel('number of targets','fontsize',11)
%   ylabel('worst-case attack rate','fontsize',11) 
    
  
  % compare the communication 
  figure (2); hold on;grid on;
  
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_resi(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
 
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)  

  shadedErrorBar(min_num_bots : max_num_bots, com_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)

  shadedErrorBar(min_num_bots : max_num_bots, com_cliq_form(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','--b', 'patchSaturation',0.33)

  
%   shadedErrorBar(min_num_bots : max_num_bots, com_dis_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops','--g', 'patchSaturation',0.33)
  
%   shadedErrorBar(min_num_bots : max_num_bots, com_dis_obv(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b', 'patchSaturation',0.33)  
  
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
%g

  title('comparison of communication messages','fontsize',12)
  legend('distributed resilient', 'distributed resilient average',...
      'clique cover', 'clique cover average', 'FontSize',14);
  xlabel('Number of robots','fontsize',14)
  ylabel('Number of communications','fontsize',14) 
          
  
  % compare the running time
  figure (3); hold on;grid on;
  
  shadedErrorBar(min_num_bots : max_num_bots, t_cen_resi(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', '--b','patchSaturation',0.33)
 
  shadedErrorBar(min_num_bots : max_num_bots, t_cen_gre(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':g','patchSaturation',0.33)    
  
  shadedErrorBar(min_num_bots : max_num_bots, t_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, t_cliq_form(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-.m', 'patchSaturation',0.33)

  
%   shadedErrorBar(min_num_bots : max_num_bots, t_dis_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops','--g', 'patchSaturation',0.33) 

%   shadedErrorBar(min_num_bots : max_num_bots, eva_dis_obv(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b', 'patchSaturation',0.33)    

%   shadedErrorBar(min_num_bots : max_num_bots, eva_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, eva_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
%g

  title('comparison of computational time','fontsize',12)
  legend('centralized resilient', 'centralized resilient average', ...
      'centralized greedy', 'centralized greedy average',...
      'distributed resilient', 'distributed resilient average',...
      'clique cover', 'clique cover average','FontSize',14);
  xlabel('Number of robots','fontsize',14)
  ylabel('Running time','fontsize',14)
  
  % plot the number of cliques
  figure (4); hold on; grid on; 
  shadedErrorBar(min_num_bots : max_num_bots, n_cliqs_G(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)
  shadedErrorBar(min_num_bots : max_num_bots, cliq_num_G(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','--b', 'patchSaturation',0.33)   
  title('number of cliqs and cliq number','fontsize',12)
  legend('number of cliques', 'number of cliques average', 'clique number', ...
      'clique number average', 'FontSize',14);
  xlabel('Number of robots','fontsize',14)
  ylabel('Number of cliques and clique number','fontsize',14)
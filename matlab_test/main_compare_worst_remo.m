% main file, compare different strategies with different strategies
% compare 5 stratgies: 
% fully distributed,
% hierarchcal_centralized_oblivious
% hierarchcal_centralized_oblivious_greedy
% centralized resilient
% centralized greedy
clear all;
global Nr N_tra Nt N_atk nei_range track_range

Nt = 50;

N_trials = 10;
  
% the # of robots, 
min_num_bots = 20; 
max_num_bots = 25; 

% the number of trajectories 
N_tra = 4; 

% the # of worst-case attacks
N_atk = 4; 

% neighbor range, within the distance, be carefully!!!
nei_range = 50;

% target tracking range
track_range = 10; 

% the range of robot position
pos_range = 200;


for Nr =  min_num_bots : max_num_bots
    robot_set_G = (1 : Nr);
    for i = 1 : N_trials
        r_pos = rand(Nr,2)*pos_range;
        tar_pos = rand(Nt,2)*pos_range;
    
    % obtain a set of targets tracked by each trajectory
    [tar_set_rtra, ~, n_tars_max_rtra, max_rtra_inx] = robot_tra_cover_fun(r_pos, tar_pos);
    
    % centralized resilient algorithm
    [n_cen_resi(i, Nr), rate_cen_resi(i, Nr), com_cen_resi(i, Nr), eva_cen_resi(i, Nr)] = ...
        cen_resi_fun(robot_set_G, tar_set_rtra, n_tars_max_rtra, max_rtra_inx);
    
%     % centralized greedy algorithm
%     [n_cen_gre(i, Nr), rate_cen_gre(i, Nr), com_cen_gre(i, Nr), eva_cen_gre(i, Nr)] = ...
%         cen_gre_fun(robot_set_G, tar_set_rtra); 
   
    % fully distribtued resilient
    [n_dis_resi(i, Nr), rate_dis_resi(i, Nr), ...
        numofcliqs_G(i, Nr), cliqnum_G(i, Nr), com_dis_resi(i, Nr), eva_dis_resi(i, Nr)] = ...
        fully_distri_resi_fun(r_pos, tar_set_rtra, n_tars_max_rtra, max_rtra_inx);
    
    % fully distribtued greedy
    [n_dis_gre(i, Nr), rate_dis_gre(i, Nr), ...
        numofcliqs_G(i, Nr), cliqnum_G(i, Nr), com_dis_gre(i, Nr), eva_dis_gre(i, Nr)] = ...
        fully_distri_gre_fun(r_pos, tar_set_rtra);
    
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
  figure; hold on; grid on;
  shadedErrorBar(min_num_bots : max_num_bots, n_cen_resi(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
  
%   shadedErrorBar(min_num_bots : max_num_bots, n_cen_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, n_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r','patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, n_dis_gre(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-.k','patchSaturation',0.33)  

%   shadedErrorBar(min_num_bots : max_num_bots, n_hier_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, n_hier_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
% '--m', '-y'

  title('comparison of the numebr of targets tracked after worst-case attack','fontsize',12)
  legend('cen_resi', 'dis_resi', 'dis_gre');
%   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');  
  xlabel('number of robots','fontsize',11)
  ylabel('coverage number','fontsize',11)
         
  
  
  % compare the worst-case attack rate
  figure; hold on;grid on;
  
  shadedErrorBar(min_num_bots : max_num_bots, rate_cen_resi(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
 
%   shadedErrorBar(min_num_bots : max_num_bots, rate_cen_gre(:, min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-y','patchSaturation',0.33) 
  
  shadedErrorBar(min_num_bots : max_num_bots, rate_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, rate_dis_gre(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-.k', 'patchSaturation',0.33)  

%   shadedErrorBar(min_num_bots : max_num_bots, rate_hier_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, rate_hier_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
%g 
  title('comparison of removal rate','fontsize',12)
  legend('cen_resi', 'dis_resi', 'dis_gre');
%   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');
  xlabel('number of targets','fontsize',11)
  ylabel('worst-case attack rate','fontsize',11) 
    
  
  % compare the communication 
  figure; hold on;grid on;
  
  shadedErrorBar(min_num_bots : max_num_bots, com_cen_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
 
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_gre(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-y','patchSaturation',0.33)  

  shadedErrorBar(min_num_bots : max_num_bots, com_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)

  shadedErrorBar(min_num_bots : max_num_bots, com_dis_gre(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-.k', 'patchSaturation',0.33)
  
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, com_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
%g

  title('comparison of communication messages','fontsize',12)
  legend('cen_resi', 'dis_resi', 'dis_gre');
%   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');
  xlabel('number of robots','fontsize',11)
  ylabel('communication messages','fontsize',11)   
        
  
  % compare the running time
  figure; hold on;grid on;
  
  shadedErrorBar(min_num_bots : max_num_bots, eva_cen_resi(:, min_num_bots : max_num_bots),...
      {@mean,@std}, 'lineprops', ':b','patchSaturation',0.33)
 
%   shadedErrorBar(min_num_bots : max_num_bots, eva_cen_gre(:, min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-y','patchSaturation',0.33)    

  shadedErrorBar(min_num_bots : max_num_bots, eva_dis_resi(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-r', 'patchSaturation',0.33)
  
  shadedErrorBar(min_num_bots : max_num_bots, eva_dis_gre(:,min_num_bots : max_num_bots),...
      {@mean,@std},'lineprops','-.k', 'patchSaturation',0.33)  

%   shadedErrorBar(min_num_bots : max_num_bots, eva_cen_o(:,min_num_bots : max_num_bots),...
%       {@mean,@std},'lineprops',':b','patchSaturation',0.33);
% 
%   shadedErrorBar(min_num_bots : max_num_bots, eva_cen_og(:,min_num_bots : max_num_bots),...
%       {@mean,@std}, 'lineprops', '-.k','patchSaturation',0.33)
%g

  title('comparison of number of evaluations','fontsize',12)
  legend('cen_resi', 'dis_resi', 'dis_gre');
%   legend('ful_dis', 'hier_cen_o', 'hier_cen_og', 'cen_resi', 'cen_gre');
  xlabel('number of robots','fontsize',11)
  ylabel('number of evaluations','fontsize',11)   
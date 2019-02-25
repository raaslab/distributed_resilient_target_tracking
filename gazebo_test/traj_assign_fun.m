%trajectory assign algorithm, include both bruturesilient

function traj_assign_fun(~,~)

global T cnt N_uavs N_tars uavs_pos uavs_ini_pos  gz_model_states_sub ...
       uav1_state_sub uav2_state_sub uav3_state_sub uav4_state_sub uav5_state_sub...
       uav6_state_sub uav7_state_sub uav8_state_sub uav9_state_sub uav10_state_sub...
       tars_pos tars_pos_hat  tars_sigma  tars_pos_last  tars_t_last  tars_v markerPub marker

 cnt = cnt + 1; 
 
 uavs_pos_local = zeros(N_uavs,3);

%receive the gazebo_model_states
 gz_model_states = receive(gz_model_states_sub,3); 


% receive the uav pos
uav1_state = receive(uav1_state_sub,1);
uav2_state = receive(uav2_state_sub,1);
uav3_state = receive(uav3_state_sub,1);
uav4_state = receive(uav4_state_sub,1);
uav5_state = receive(uav5_state_sub,1);
uav6_state = receive(uav6_state_sub,1);
uav7_state = receive(uav7_state_sub,1);
uav8_state = receive(uav8_state_sub,1);
uav9_state = receive(uav9_state_sub,1);
uav10_state = receive(uav10_state_sub,1);

%calculate the uav local states from mavros receiving
uavs_pos_local(1,:) = [uav1_state.Pose.Position.X, uav1_state.Pose.Position.Y, uav1_state.Pose.Position.Z];
uavs_pos_local(2,:) = [uav2_state.Pose.Position.X, uav2_state.Pose.Position.Y, uav2_state.Pose.Position.Z];
uavs_pos_local(3,:) = [uav3_state.Pose.Position.X, uav3_state.Pose.Position.Y, uav3_state.Pose.Position.Z];
uavs_pos_local(4,:) = [uav4_state.Pose.Position.X, uav4_state.Pose.Position.Y, uav4_state.Pose.Position.Z];
uavs_pos_local(5,:) = [uav5_state.Pose.Position.X, uav5_state.Pose.Position.Y, uav5_state.Pose.Position.Z];
uavs_pos_local(6,:) = [uav6_state.Pose.Position.X, uav6_state.Pose.Position.Y, uav6_state.Pose.Position.Z];
uavs_pos_local(7,:) = [uav7_state.Pose.Position.X, uav7_state.Pose.Position.Y, uav7_state.Pose.Position.Z];
uavs_pos_local(8,:) = [uav8_state.Pose.Position.X, uav8_state.Pose.Position.Y, uav8_state.Pose.Position.Z];
uavs_pos_local(9,:) = [uav9_state.Pose.Position.X, uav9_state.Pose.Position.Y, uav9_state.Pose.Position.Z];
uavs_pos_local(10,:) = [uav10_state.Pose.Position.X, uav10_state.Pose.Position.Y, uav10_state.Pose.Position.Z];

uavs_pos = uavs_pos_local + uavs_ini_pos; 
 
%calcaulate the positions of the N_uavs targets, use services
%  [uavs_pos(1,:), ~, ~] = getState(uav1_state); 
%  [uavs_pos(2,:), ~, ~] = getState(uav2_state);
%  [uavs_pos(3,:), ~, ~] = getState(uav3_state);
%  [uavs_pos(4,:), ~, ~] = getState(uav4_state);
%[uavs_pos(5,:), ~, ~] = getState(uav5_state);

% for targets !!! make sure to check if the ID corresponds
% well!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
%we use a global sensor to measure pos of the target
 for i = 1: N_tars
     
     %true position of the targets
     tars_pos(i,:) = [gz_model_states.Pose(i+1).Position.X, ...
                         gz_model_states.Pose(i+1).Position.Y,...
                         gz_model_states.Pose(i+1).Position.Z] + normrnd(0, 0.1, 1,3);                   
     %update  the 2D pos of the target by KF filter. Notably, the z
     %coordinate is always 0. 
     
% %  %specify for each target. 
%       [tars_pos_hat(i, 1:2), tars_sigma(2*i -1:2*i, :), tars_pos_last(i, 1:2), tars_t_last(i,:),  tars_v(i, 1:2)] = ...
%           tar_pos_update(tars_pos_true(i, 1:2), tars_pos_hat(i, 1:2), tars_sigma(2*i -1:2*i, :), tars_pos_last(i, 1:2), tars_t_last(i,:), T*cnt, tars_v(i,1:2));                
 end
 
% we assume that the measurements have Gaussian noise
% and we average ten of these measurements

%  uavs_pos(4,:) = [gz_model_states.Pose(N_targets+2).Position.X,...
%                   gz_model_states.Pose(N_targets+2).Position.Y,...
%                   gz_model_states.Pose(N_targets+2).Position.Z];
%               
%  uavs_pos(1,:) = [gz_model_states.Pose(N_targets+3).Position.X,...
%                   gz_model_states.Pose(N_targets+3).Position.Y,...
%                   gz_model_states.Pose(N_targets+3).Position.Z];  
%               
%  uavs_pos(3,:) = [gz_model_states.Pose(N_targets+4).Position.X,...
%                   gz_model_states.Pose(N_targets+4).Position.Y,...
%                   gz_model_states.Pose(N_targets+4).Position.Z];
%               
%  uavs_pos(2,:) = [gz_model_states.Pose(N_targets+5).Position.X,...
%                   gz_model_states.Pose(N_targets+5).Position.Y,...
%                   gz_model_states.Pose(N_targets+5).Position.Z];                
         
 

 %***after receive the pos from uavs and targets
 [target_cover, n_id_maxtra] = uav_tra_cover_fun();
 
 
 %%trajectory selection by four different algorithms
 % brute_force, resilient, greedy and random

 % 1st brute_force
  % bf_traj_assign(target_cover);
 
 % 2nd resilient 
 % resilient_traj_assign(target_cover, n_id_maxtra);  
  
 % distributed resilient
  distri_resi_traj_assign(target_cover, n_id_maxtra)
 
 % distributed greedy
 % greedy_traj_assign(target_cover);
 %distri_gre_traj_assign(target_cover)
  
 % 4th random
  % random_traj_assign(target_cover);
 end
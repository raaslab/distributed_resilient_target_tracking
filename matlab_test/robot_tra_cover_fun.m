%  function: calculate a set of targets tracked by each trajectory
function [tar_set_rtra, n_tars_rtra, n_tars_max_rtra, max_rtra_inx] ...
    = robot_tra_cover_fun(r_pos, t_pos)
    
    global Nr Nt N_tra track_range

    
    n_tars_max_rtra = zeros(Nr,1); % select the maximum coverage of it trajectories for each robot
    max_rtra_inx = zeros(Nr,1); %  the max tra index
    tar_set_rtra = cell(Nr, N_tra); % The targets can be covered for a specific robot with a choosing trajecotry
    n_tars_rtra= zeros(Nr, N_tra); % The number of targets covered for a specific robot with a choosing trajectory

        for i = 1 : Nr
             for j = 1 : N_tra

                   if j == 1 % up_trajectory
                      for k = 1 : Nt % check all the targets
                           if t_pos(k,2) >= r_pos(i,2) && abs(r_pos(i,1)-t_pos(k,1))<=track_range % the targets are above the robot, just use ||x_r-x_t||
                               tar_set_rtra{i,j}=[tar_set_rtra{i,j}, k]; % store the targets can be tracked if the distance is within tolerance
                           end
                      end


                   elseif j == 2 % down_trajectory 
                       for k = 1:Nt 
                           if t_pos(k,2) <= r_pos(i,2) && abs(r_pos(i,1)-t_pos(k,1))<=track_range % targets are below, ||x_r-x_t||
                               tar_set_rtra{i,j}=[tar_set_rtra{i,j}, k];
                           end
                       end

                   elseif j == 3 % left_trajectory
                       for k = 1 : Nt 
                           if t_pos(k,1) <= r_pos(i,1) && abs(r_pos(i,2)-t_pos(k,2))<=track_range % targets are at left, ||y_r-y_t||
                               tar_set_rtra{i,j}=[tar_set_rtra{i,j}, k];
                           end
                       end

                   else % right-trajectory
                       for k = 1 : Nt 
                           if t_pos(k,1) >= r_pos(i,1) && abs(r_pos(i,2)-t_pos(k,2))<=track_range % targets are at right, ||y_r-y_t||
                               tar_set_rtra{i,j}=[tar_set_rtra{i,j}, k];
                           end
                       end

                   end

            n_tars_rtra(i,j) = length(tar_set_rtra{i,j});
             end % j indicates the direction
             % pick the maximum trajecotry (Num_tarcover) and its corresponding targets for each robot 
             % max trajectory for each robot
            [n_tars_max_rtra(i), max_rtra_inx(i)]= max(n_tars_rtra(i,:)); 
        end
    
end
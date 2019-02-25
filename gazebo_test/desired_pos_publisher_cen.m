% publish the desired pos for each uav in ros-network
% we first calculate the desired pos for each uav in gazebo global pos
% then transform to a local desired pos for python.receiver for control
function desired_pos_publisher_cen(traj_assign, n_after_remo, cliq_id,  t_run, com)

    global uavs_ini_pos 
    
    global uavs_pos
    
    global N_uavs
    
    global fly_length
    
    global desired_pos_pub_uavs desired_pub_msg err_offset desired_pos_track
    
    global store_n_after_remo store_t_run store_com  
        
    global work_status_pub_uavs work_status_uavs cliq_id_pub_uavs cliq_id_uavs
        
    global opt_attacked_uavs
    
    global cnt
    
    % each uav has a desired x, y ,z
    desired_pos_global = zeros(N_uavs, 3); % in the gazebo on global frame
    desired_pos_local = zeros(N_uavs,3); % to publish to uav on local frame
    
    for i = 1 : N_uavs
        
        if traj_assign(i) ==1 % go up 
            
            %first calculate the desired pos in global gazebo frame
            desired_pos_global(i,1)=uavs_pos(i,1);
            desired_pos_global(i,2)=uavs_pos(i,2) + fly_length; % go up
           
                   
        elseif traj_assign(i) ==2 % go down
            
            %first calculate the desired pos in global gazebo frame
            desired_pos_global(i,1)=uavs_pos(i,1);
            desired_pos_global(i,2)=uavs_pos(i,2) - fly_length; % go down
                                                 
        elseif traj_assign(i) ==3 % go right
            
            %first calculate the desired pos in global gazebo frame
            desired_pos_global(i,1)=uavs_pos(i,1) + fly_length; %go right
            desired_pos_global(i,2)=uavs_pos(i,2); % go up
                                                      
        else % go left
             
            %first calculate the desired pos in global gazebo frame
            desired_pos_global(i,1)=uavs_pos(i,1) - fly_length; % go left
            desired_pos_global(i,2)=uavs_pos(i,2);
                                      
        end
                
        
    end
    
    if  mod(cnt, 10) == 0
%         norm(uavs_pos(8,:) - desired_pos_track(8,:))<= err_offset &&...
%         norm(uavs_pos(9,:) - desired_pos_track(9,:))<= err_offset &&...
%         norm(uavs_pos(10,:) - desired_pos_track(10,:))<= err_offset 
%         norm(uavs_pos(1,:) - desired_pos_track(1,:))<= err_offset &&...
%         norm(uavs_pos(2,:) - desired_pos_track(2,:))<= err_offset &&...
%         norm(uavs_pos(3,:) - desired_pos_track(3,:))<= err_offset &&...
%         norm(uavs_pos(4,:) - desired_pos_track(4,:))<= err_offset &&...
%         norm(uavs_pos(5,:) - desired_pos_track(5,:))<= err_offset &&...
%         norm(uavs_pos(6,:) - desired_pos_track(6,:))<= err_offset &&...
%         norm(uavs_pos(7,:) - desired_pos_track(7,:))<= err_offset &&...
        desired_pos_track(:,1:2) = desired_pos_global(:,1:2);
        
        for i = 1 : N_uavs 
           if ismember(i, opt_attacked_uavs) == 1
               work_status_uavs(i).Data = 0; 
           else 
               work_status_uavs(i).Data = 1; 
           end
           % give cliq_id for each uav
           cliq_id_uavs(i).Data = cliq_id(i); 
        end
        %also store the data
        store_n_after_remo = [store_n_after_remo, n_after_remo];
%         store_n_cliqs  = [store_n_cliqs, n_cliqs];
%         store_cliq_num = [store_cliq_num, cliq_num]; 
        store_t_run = [store_t_run, t_run];
        store_com = [store_com, com]; 
        
        %doing some plots to show tar_before_removal, tar_after_remove,
        %remove_rate           
         figure(1); 
         cla; hold on;
%         subplot(3,1,1);
%         plot(store_tar_before_remove, '--ro', 'LineWidth',2,...
%                        'MarkerEdgeColor','g',...
%                        'MarkerFaceColor','g',...
%                        'MarkerSize',10)
%         title('Number of targets tracked before optimal removal')
% 
%         subplot(3,1,2); 
        plot(store_n_after_remo,'rs', 'LineWidth',2,...
                       'MarkerEdgeColor','r',...
                       'MarkerFaceColor','r',...
                       'MarkerSize',8)       
        title('Number of remaining targets after optimal removal')
%         
%         subplot(3,1,3); 
%         plot(store_remove_rate,'--r*', 'LineWidth',2,...
%                        'MarkerEdgeColor','m',...
%                        'MarkerFaceColor','m',...
%                        'MarkerSize',10)
%         title(' Optimal removal rate')
%         
         hold off;
        
    else  % the robots havn't gone to the desired positions 

    end
     
    for i = 1 : N_uavs

        %then calculate the desired pos in local gazebo frame 
        desired_pos_local(i,:) = desired_pos_track(i,:) - uavs_ini_pos(i,:);

        %once we have the local desired pos, publish it on uav, this
        %message is waiting by a .py running code
        desired_pub_msg.X = desired_pos_local(i,1);
        desired_pub_msg.Y = desired_pos_local(i,2);
        desired_pub_msg.Z = desired_pos_track(i,3); % the height needs not change

        send(desired_pos_pub_uavs(i), desired_pub_msg);        
            
        %publish the uav_work status
        send(work_status_pub_uavs(i), work_status_uavs(i));    
        
        %publish cliq id for each uav
        send(cliq_id_pub_uavs(i), cliq_id_uavs(i)); 

    end
          
    
end
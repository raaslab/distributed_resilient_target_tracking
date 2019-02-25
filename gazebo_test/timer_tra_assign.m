%Timer: constantly receive the pos of all the models in the gazebo and pick the
%trajectory for each uav
cla; clear all;
%subscribe the gazebo pose topic to get the pos_info of each uav and each
%ground target
global T cnt

global N_uavs N_tars N_fail_uavs N_resilience_uavs

global nei_range

global uavs_pos 

global uavs_ini_pos %each uav has a initial pos which is used to calcualte the local pos diff

global N_dir_uav %1, up; 2, down; 3, right; 4 left. It has four directions

global track_length track_width fly_length sta_track_length

global uav_id_set

global gz_model_states_sub

global desired_pos_pub_uavs desired_pub_msg work_status_pub_uavs work_status_uavs...
        cliq_id_pub_uavs cliq_id_uavs

global uav1_state_sub uav2_state_sub uav3_state_sub uav4_state_sub uav5_state_sub ...
        uav6_state_sub uav7_state_sub uav8_state_sub uav9_state_sub uav10_state_sub...
        desired_pos_track

global err_offset
    
global tars_pos_true tars_pos_hat  tars_sigma  tars_pos_last  tars_t_last  tars_v

global store_n_after_remo store_n_cliqs store_cliq_num store_t_run store_com

%global desired_pos_global


N_uavs = 10; % number of uavs 
N_tars = 50; % number of targets

N_fail_uavs = 4; % number of failed uavs
N_resilience_uavs = N_uavs - N_fail_uavs; %number of resilient uavs

% communication, neighbor range for the clique formulation
nei_range = 8; 

tars_pos_true = zeros(N_tars, 3); % x, y, z
tars_pos_hat = zeros(N_tars, 3);  %may have some problem!!!!!!
tars_pos_last = zeros(N_tars, 3);  %may have some problem!!!!!!
tars_t_last = zeros(N_tars, 1);
tars_v = zeros(N_tars, 3);
tars_sigma = zeros(2*N_tars, 2); 
for i =  1 : N_tars
    tars_sigma(2*i-1 :2*i, : )  = eye(2); 
end

err_offset = 1.8;

store_n_after_remo = [];
store_n_cliqs =[];
store_cliq_num =[];
store_t_run = [];
store_com = [];

%uavs_ini_pos = [0, -5, 2; 3, -2, 3; 5, 0, 4; -2, 5, 5; -5, 0, 6];
%uavs_ini_pos = [0, -5, 2; 5, 0, 3; -2, 5, 4; -5, 0, 5];
%uavs_ini_pos = [-3, -5, 2; -2, -5 3; -1,-5, 4]; 
uavs_ini_pos = [0, -7, 0; 2, -7, 0; 4, -6, 0; 0, 8, 0; 6, -4, 0; ...
    2, 8, 0; -6, 0, 0; -4, -5, 0; 7, 0, 0; 5, -2, 0];

uavs_pos = zeros(N_uavs,3); %x, y ,z

desired_pos_track = [0, -7, 1.5; 2, -7, 2; 4, -6, 2.5; 0, 8, 3; 6, -4, 3.5; ...
    2, 8, 4; -6, 0, 4.5; -4, -5, 5; 7, 0, 5.5; 5, -2, 6];

N_dir_uav = 4; % each uav has four trajectories it can choose

%assume the uav has a 3x2 rectangle when it is static
track_width  = 3; % tracking the target within the width along that direction
sta_track_length  = 3; 
fly_length = 3; 
track_length = fly_length + sta_track_length; % track the target within the length in that direction, static cover plus moving 

uav_id_set = zeros(1,N_uavs);

for i =1:N_uavs
    uav_id_set(i)=i;
end

%publish desired pos for each uav 
desired_pos_pub_uav1 = rospublisher('/uav1/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav2 = rospublisher('/uav2/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav3 = rospublisher('/uav3/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav4 = rospublisher('/uav4/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav5 = rospublisher('/uav5/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav6 = rospublisher('/uav6/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav7 = rospublisher('/uav7/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav8 = rospublisher('/uav8/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav9 = rospublisher('/uav9/desired_pos','geometry_msgs/Point');
desired_pos_pub_uav10 = rospublisher('/uav10/desired_pos','geometry_msgs/Point');

desired_pos_pub_uavs = [desired_pos_pub_uav1, desired_pos_pub_uav2, ...
    desired_pos_pub_uav3, desired_pos_pub_uav4, desired_pos_pub_uav5, desired_pos_pub_uav6, ...
    desired_pos_pub_uav7, desired_pos_pub_uav8, desired_pos_pub_uav9, desired_pos_pub_uav10];

desired_pub_msg = rosmessage('geometry_msgs/Point'); 


%publish the working_status for each uav 
work_staus_pub_uav1 = rospublisher('/uav1/work_status', 'std_msgs/Int8');
work_staus_pub_uav2 = rospublisher('/uav2/work_status', 'std_msgs/Int8');
work_staus_pub_uav3 = rospublisher('/uav3/work_status', 'std_msgs/Int8');
work_staus_pub_uav4 = rospublisher('/uav4/work_status', 'std_msgs/Int8');
work_staus_pub_uav5 = rospublisher('/uav5/work_status', 'std_msgs/Int8');
work_staus_pub_uav6 = rospublisher('/uav6/work_status', 'std_msgs/Int8');
work_staus_pub_uav7 = rospublisher('/uav7/work_status', 'std_msgs/Int8');
work_staus_pub_uav8 = rospublisher('/uav8/work_status', 'std_msgs/Int8');
work_staus_pub_uav9 = rospublisher('/uav9/work_status', 'std_msgs/Int8');
work_staus_pub_uav10 = rospublisher('/uav10/work_status', 'std_msgs/Int8');

work_status_pub_uavs = [work_staus_pub_uav1, work_staus_pub_uav2, ...
    work_staus_pub_uav3, work_staus_pub_uav4, work_staus_pub_uav5, work_staus_pub_uav6...
    work_staus_pub_uav7, work_staus_pub_uav8, work_staus_pub_uav9, work_staus_pub_uav10]; 

work_status_uav_1  = rosmessage('std_msgs/Int8');
work_status_uav_2  = rosmessage('std_msgs/Int8');
work_status_uav_3  = rosmessage('std_msgs/Int8');
work_status_uav_4  = rosmessage('std_msgs/Int8');
work_status_uav_5  = rosmessage('std_msgs/Int8');
work_status_uav_6  = rosmessage('std_msgs/Int8');
work_status_uav_7  = rosmessage('std_msgs/Int8');
work_status_uav_8  = rosmessage('std_msgs/Int8');
work_status_uav_9  = rosmessage('std_msgs/Int8');
work_status_uav_10  = rosmessage('std_msgs/Int8');
work_status_uavs = [work_status_uav_1, work_status_uav_2, work_status_uav_3, work_status_uav_4,...
    work_status_uav_5, work_status_uav_6, work_status_uav_7, work_status_uav_8, ...
    work_status_uav_9, work_status_uav_10]; 

%publish the cliq ID for each uav 
cliq_id_pub_uav1 = rospublisher('/uav1/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav2 = rospublisher('/uav2/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav3 = rospublisher('/uav3/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav4 = rospublisher('/uav4/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav5 = rospublisher('/uav5/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav6 = rospublisher('/uav6/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav7 = rospublisher('/uav7/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav8 = rospublisher('/uav8/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav9 = rospublisher('/uav9/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uav10 = rospublisher('/uav10/cliq_id', 'std_msgs/Int8');
cliq_id_pub_uavs = [cliq_id_pub_uav1, cliq_id_pub_uav2, cliq_id_pub_uav3, cliq_id_pub_uav4, ...
    cliq_id_pub_uav5, cliq_id_pub_uav6, cliq_id_pub_uav7, cliq_id_pub_uav8, ...
    cliq_id_pub_uav9, cliq_id_pub_uav10]; 

cliq_id_uav_1  = rosmessage('std_msgs/Int8');
cliq_id_uav_2  = rosmessage('std_msgs/Int8');
cliq_id_uav_3  = rosmessage('std_msgs/Int8');
cliq_id_uav_4  = rosmessage('std_msgs/Int8');
cliq_id_uav_5  = rosmessage('std_msgs/Int8');
cliq_id_uav_6  = rosmessage('std_msgs/Int8');
cliq_id_uav_7  = rosmessage('std_msgs/Int8');
cliq_id_uav_8  = rosmessage('std_msgs/Int8');
cliq_id_uav_9  = rosmessage('std_msgs/Int8');
cliq_id_uav_10  = rosmessage('std_msgs/Int8');
cliq_id_uavs = [cliq_id_uav_1, cliq_id_uav_2, cliq_id_uav_3, cliq_id_uav_4, cliq_id_uav_5,...
    cliq_id_uav_6, cliq_id_uav_7, cliq_id_uav_8, cliq_id_uav_9, cliq_id_uav_10]; 


%subscribe all the model states in the gazebo environment
gz_model_states_sub = rossubscriber('/gazebo/model_states');

%global markerPub marker
%markerPub = rospublisher('/visualization_marker','visualization_msgs/Marker');
%marker = rosmessage(markerPub);

%%%receive ros_gazebo service
% gazebo = ExampleHelperGazeboCommunicator();
% 
% uav1_state = ExampleHelperGazeboSpawnedModel('iris_1',gazebo);
% uav2_state = ExampleHelperGazeboSpawnedModel('iris_2',gazebo);
% uav3_state = ExampleHelperGazeboSpawnedModel('iris_3',gazebo);
% uav4_state = ExampleHelperGazeboSpawnedModel('iris_4',gazebo);
%uav5_state = ExampleHelperGazeboSpawnedModel('iris_5',gazebo);

%get the uav pos from mavros

uav1_state_sub = rossubscriber('/uav1/mavros/local_position/pose');
uav2_state_sub = rossubscriber('/uav2/mavros/local_position/pose');
uav3_state_sub = rossubscriber('/uav3/mavros/local_position/pose');
uav4_state_sub = rossubscriber('/uav4/mavros/local_position/pose');
uav5_state_sub = rossubscriber('/uav5/mavros/local_position/pose');
uav6_state_sub = rossubscriber('/uav6/mavros/local_position/pose');
uav7_state_sub = rossubscriber('/uav7/mavros/local_position/pose');
uav8_state_sub = rossubscriber('/uav8/mavros/local_position/pose');
uav9_state_sub = rossubscriber('/uav9/mavros/local_position/pose');
uav10_state_sub = rossubscriber('/uav10/mavros/local_position/pose');

% start the timer
T = 0.1; % timer period
cnt  = 0; 

t = timer('TimerFcn',{@traj_assign_fun},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);



%restore all the data. 
n = 50;

store_n_tar = zeros(3,n);
store_run_time = zeros(4,n);

%
load('centralized_resilient3.mat')
store_n_tar(1,1:n) = store_n_after_remo(1:n);
store_run_time(1,1:n) = store_t(1:n); 

load('centralized_greedy3.mat');
store_n_tar(2,1:n) = store_n_after_remo(1:n);
store_run_time(2,1:n) = store_t(1:n);


store_com_cliq = zeros(4, n);
%store_cliq = zeros(2, n);

load('distributed_resilient3.mat');
store_n_tar(3,1:n) = store_n_after_remo(1:n);
% store_n_tar(3,1) = 11;

store_run_time(3,1:n) = store_t(1:n);
store_run_time(4,1:n) =store_t_cliq(1:n);

store_com_cliq(1,1:n) = store_com(1:n);
store_com_cliq(2,1:n) = store_com_cliq(1:n);

store_com_cliq(3,1:40) = store_n_cliqs(1:40);
store_com_cliq(4,1:40) = store_cliq_num(1:40);

figure (1)
boxplot(store_n_tar') 

figure (2)
boxplot(store_run_time')

figure (3)
boxplot(store_com_cliq(1:2, :)')

figure (4)
boxplot(store_com_cliq(3:4, :)')
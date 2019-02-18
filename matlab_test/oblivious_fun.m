% function: assign trajectory to robot by an oblivious algorithm
% pick out the first alpha biggest ones. 
function [r_tra_o, eva_o]  = oblivious_fun(r_set, alpha, n_tars_max_rtra, max_rtra_inx)
    
    global N_tra
    % store the number of evaluations of sorting
    eva_o = length(r_set) * N_tra * log2(length(r_set) * N_tra);
    % select the first alpha biggest alpha ones
    % create a matrix with r_set and n_tars_max_rtra
    r_n_tars = zeros(length(r_set), 2);
    for i = 1 : length(r_set)
        r_n_tars(i, :) = [r_set(i), n_tars_max_rtra(r_set(i))];
    end
    % after getting r_n_tars, sort the matrix in a 'descend' order 
    % as the second row
    [~, inx] = sort(r_n_tars(:,2), 'descend');
    r_n_tars_sort = r_n_tars(inx, :); 
    % then we pick out the first alpha in the first row
    r_tra_o = zeros(alpha, 2);
    for i = 1 : alpha 
        r_tra_o(i, :) = [r_n_tars_sort(i,1), max_rtra_inx(r_n_tars_sort(i,1))]; 
    end
end
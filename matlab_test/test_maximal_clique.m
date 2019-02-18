%test to find the maximal cliques of adjacency matrix
A = [0 1 1 1 1 0; 1 0 1 0 0 0; 1 1 0 0 0 0;...
    1 0 0 0 1 1; 1 0 0 1 0 1; 0 0 0 1 1 0];
A2 = [0 1 1 0 0 0; 1 0 1 1 1 0; 1 1 0 0 0 0;...
    0 1 0 0 1 1; 0 1 0 1 0 0; 0 0 0 1 0 0];
A3 = [0 1 0; 1 0 0; 0 0 0];

cliques = maximalCliques(A3, 'v2'); 

% [~,c,~] = k_clique(3, A);  
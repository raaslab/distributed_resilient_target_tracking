% function: the # of targets covered by this strategy
function [n_cover] = n_tra_cover(tar_set_rtra, r_tra)     
   global Nr
   s_tra_cover = cell(Nr+1,1); %the target covered by this strategy
   for i = 1 : Nr
   s_tra_cover{i+1} = union(s_tra_cover{i}, tar_set_rtra{r_tra(i,1), r_tra(i,2)});
   end
   n_cover = length(s_tra_cover{Nr+1});           
end
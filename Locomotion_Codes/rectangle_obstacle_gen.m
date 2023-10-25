obs_index = [1 3 ; 2 3 ; 2 4 ; 1 4 ; 1 3];

for i = 1:length(rectangle)
    obstacle{i} = rectangle{i}(obs_index);
end
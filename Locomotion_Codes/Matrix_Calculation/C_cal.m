function C = C_cal(fixed_node,n)

C = zeros(1,3*length(n.pos));
index = 1;

for k = 1:length(fixed_node)
    C(index,fixed_node(k)) = 1; % x_coord
    C(index+1,length(n.pos) + fixed_node(k)) = 1; % y_coord
    C(index+2,2*length(n.pos) + fixed_node(k)) = 1; % z_coord
    index = index + 3;
end

end
%% 
% Manipulate intermediate configuration so that it can stand on the flat ground
plot_truss(truss16_org);axis tight
view([70, 20]); title('intermediate truss')
    
truss_intermed = truss16_org;
truss_intermed.pos(3,[2 3 4 5]) = -1;
truss_intermed.pos(3,7) = -0.2;
truss_intermed.pos(3,[6 8]) = truss_intermed.pos(3,[6 8]) + 0.8;
truss_intermed.pos(:,1) = mean(truss_intermed.pos(:,[1 7]), 2);
if check_constraints_truss(truss_intermed, truss_intermed.pos)
    plot_truss(truss_intermed);axis tight
    view([70, 20]); title('intermediate truss')
end

%%  
close all;
% Manipulate tower configuration so that it is natural to 
% convert intermediate configuration into tower configuration 
truss_tower(1) = truss_intermed;
truss_tower(1).elist = tall300.elist;
truss_tower(1).edge_adj = tall300.edge_adj;
truss_tower(1).adj = tall300.adj;
truss_tower(1).pos = truss_intermed.pos;

truss_tower(1).pos(:,9) = truss_tower(1).pos(:,6);
truss_tower(1).pos(3,9) = truss_tower(1).pos(3,6) - 0.1;

if check_constraints_truss(truss_tower(1), truss_tower(1).pos)
    plot_truss(truss_tower(1)); axis tight
    view([70, 20]); title('Toward tower step 1')
end

truss_tower(2) = truss_intermed;
truss_tower(2).elist = tall300.elist;
truss_tower(2).edge_adj = tall300.edge_adj;
truss_tower(2).adj = tall300.adj;
truss_tower(2).pos = truss_intermed.pos;

truss_tower(2).pos(:,9) = truss_tower(2).pos(:,6);
truss_tower(2).pos(3,9) = -1;

if check_constraints_truss(truss_tower(2), truss_tower(2).pos)
    plot_truss(truss_tower(2)); axis tight
    view([70, 20]); title('Toward tower step 2')
end

truss_tower(3) = truss_tower(2);
truss_tower(3).pos([1 2], 6) = mean(truss_tower(2).pos([1 2], [2 4]), 2);

if check_constraints_truss(truss_tower(3), truss_tower(3).pos)
    plot_truss(truss_tower(3)); axis tight
    view([70, 20]); title('Toward tower step 3')
end

truss_tower(4) = truss_tower(2);
truss_tower(4).pos([1 2], 1) = truss_tower(4).pos([1 2], 5);
truss_tower(4).pos(3, 1) = truss_tower(4).pos(3, 5) + 2;
truss_tower(4).pos([1 2], 8) = truss_tower(4).pos([1 2], 3);
truss_tower(4).pos(3, 8) = truss_tower(4).pos(3, 3) + 2;
truss_tower(4).pos([1 2], 7) = truss_tower(4).pos([1 2], 9);
truss_tower(4).pos(3, 7) = truss_tower(4).pos(3, 9) + 2;
truss_tower(4).pos([1 2], 4) = truss_tower(4).pos([1 2], 1);
truss_tower(4).pos(3, 4) = truss_tower(4).pos(3, 1) + 2;
truss_tower(4).pos([1 2], 6) = truss_tower(4).pos([1 2], 8);
truss_tower(4).pos(3, 6) = truss_tower(4).pos(3, 8) + 2;
truss_tower(4).pos([1 2], 2) = truss_tower(4).pos([1 2], 7);
truss_tower(4).pos(3, 2) = truss_tower(4).pos(3, 7) + 2;

if check_constraints_truss(truss_tower(4), truss_tower(4).pos)
    plot_truss(truss_tower(4)); axis tight
    view([70, 20]); title('Toward tower step 3')
end
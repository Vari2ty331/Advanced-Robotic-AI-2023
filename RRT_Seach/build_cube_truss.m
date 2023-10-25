%% directly moving toward cube configuration is impossible!
truss_intermed_cube(1) = truss_intermed;
truss_intermed_cube(1).elist = cube298.elist;
truss_intermed_cube(1).edge_adj = cube298.edge_adj;
truss_intermed_cube(1).adj = cube298.adj;
truss_intermed_cube(1).pos(:,9) = truss_intermed_cube(1).pos(:,6);
truss_intermed_cube(1).pos(3,6) = truss_intermed_cube(1).pos(3,6) + 0.2;
truss_intermed_cube(1).pos(3,9) = truss_intermed_cube(1).pos(3,9) - 0.2;
if check_constraints_truss(truss_intermed_cube(1), truss_intermed_cube(1).pos)
    plot_truss(truss_intermed_cube(1)); axis tight
    view([20, 10]); title('Toward cube directly')
end

truss_intermed_cube(2) = truss_intermed;
truss_intermed_cube(2).elist = cube298.elist;
truss_intermed_cube(2).edge_adj = cube298.edge_adj;
truss_intermed_cube(2).adj = cube298.adj;
truss_intermed_cube(2).pos(:,9) = truss_intermed_cube(2).pos(:,6);
truss_intermed_cube(2).pos(3,6) = truss_intermed_cube(2).pos(3,6) + 0.2;
truss_intermed_cube(2).pos(3,9) = -1;
if check_constraints_truss(truss_intermed_cube(2), truss_intermed_cube(2).pos)
    plot_truss(truss_intermed_cube(2)); axis tight
    view([20, 10]); title('Toward cube directly')
end

%% toward cube configuration
close all;

truss_cube(1) = truss_intermed;
truss_cube(1).elist = tall300.elist;
truss_cube(1).edge_adj = tall300.edge_adj;
truss_cube(1).adj = tall300.adj;
truss_cube(1).pos = truss_intermed.pos;


truss_cube(1).pos(1:2,6) = mean(truss_cube(1).pos(1:2,[2 4 7]),2);
truss_cube(1).pos(3,6) = -1;
truss_cube(1).pos(1:2,9) = truss_cube(1).pos(1:2,6) - 0.2;
truss_cube(1).pos(3,9) = truss_cube(1).pos(3,6);

if check_constraints_truss(truss_cube(1), truss_cube(1).pos)
    plot_truss(truss_cube(1)); axis tight
    view([70, 20]); title('Toward cube step 1')
end

truss_cube(2) = truss_cube(1);
truss_cube(2).elist = cube298.elist;
truss_cube(2).edge_adj = cube298.edge_adj;
truss_cube(2).adj = cube298.adj;
truss_cube(2).pos(3,6) = truss_cube(2).pos(3,6);
if check_constraints_truss(truss_cube(2), truss_cube(2).pos)
    plot_truss(truss_cube(2)); axis tight
    view([70, 20]); title('Toward cube step 2')
end

truss_cube(3) = truss_cube(2);
truss_cube(3).pos(1:2,9) = mean(truss_cube(2).pos(1:2,[4 5]),2);
truss_cube(3).pos(:,6) = mean(truss_cube(2).pos(:,[2 3]),2);
if check_constraints_truss(truss_cube(3), truss_cube(3).pos)
    plot_truss(truss_cube(3)); axis tight
    view([70, 20]); title('Toward cube step 3')
end

% truss_cube(4) = cube298;
% truss_cube(4).pos(1,:) = cube298.pos(3,:);
% truss_cube(4).pos(3,:) = -cube298.pos(1,:);
truss_cube(4) = truss_cube(3);
truss_cube(4).pos(1:2,8) = truss_cube(4).pos(1:2,3);
truss_cube(4).pos(3,8) = truss_cube(4).pos(3,3) + 2;
truss_cube(4).pos(1:2,4) = truss_cube(4).pos(1:2,9);
truss_cube(4).pos(3,4) = truss_cube(4).pos(3,9) + 2;
truss_cube(4).pos(1:2,2) = truss_cube(4).pos(1:2,6);
truss_cube(4).pos(3,2) = truss_cube(4).pos(3,6) + 2;
truss_cube(4).pos(1,5) = truss_cube(4).pos(1,5) - 0.1;
truss_cube(4).pos(2,5) = truss_cube(4).pos(2,5) - 0.1;
truss_cube(4).pos(1:2,1) = truss_cube(4).pos(1:2,5);
truss_cube(4).pos(3,1) = truss_cube(4).pos(3,5) + 2;
truss_cube(4).pos(:,7) = mean(truss_cube(4).pos(:, [1:6, 8]), 2);
if check_constraints_truss(truss_cube(4), truss_cube(4).pos)
    plot_truss(truss_cube(4)); axis tight
    view([70, 20]); title('Toward cube step 4')
end
%% cube <-> tower
% cube is for moving and tower is for shoring
% these two configurations should frequently change into each other
close all;
cube_and_tower_view = [30, 10];
cube_and_tower(1) = truss_cube(4);
if check_constraints_truss(cube_and_tower(1), cube_and_tower(1).pos)
    plot_truss(cube_and_tower(1)); axis tight
%     plot_truss_3D(cube_and_tower(1)); axis tight
    view(cube_and_tower_view); title('cube and tower, step 1')
end

cube_and_tower(2) = cube_and_tower(1);
cube_and_tower(2).pos(:,9) = mean(cube_and_tower(2).pos(:,[6 9]), 2);
cube_and_tower(2).pos(:,9) = cube_and_tower(2).pos(:,9) - [-0.2 0.2 0]';
cube_and_tower(2).pos(:,6) = cube_and_tower(2).pos(:,9) - [-0.03 0.03 0]';
% cube_and_tower(2).pos(:,6) = cube_and_tower(2).pos(:,9);
% cube_and_tower(2).pos(3,[2 4 7]) = cube_and_tower(1).pos(3,[2 4 7]) - 0.2;
cube_and_tower(2).elist = truss_tower(4).elist;
cube_and_tower(2).edge_adj = truss_tower(4).edge_adj;
cube_and_tower(2).adj = truss_tower(4).adj;
% cube_and_tower(2).pos(1:2,2) = cube_and_tower(2).pos(1:2,9);
% cube_and_tower(2).pos(:,6) = cube_and_tower(2).pos(:,2) + 0.2;
if check_constraints_truss(cube_and_tower(2), cube_and_tower(2).pos)
    plot_truss(cube_and_tower(2)); axis tight
%     plot_truss_3D(cube_and_tower(2)); axis tight
    view(cube_and_tower_view); title('cube and tower, step 2')
end


cube_and_tower(3) = cube_and_tower(2);
cube_and_tower(3).pos(:,2) = mean(cube_and_tower(3).pos(:,[2 7]), 2);
% cube_and_tower(3).pos(3,6) = cube_and_tower(3).pos(3,2) + 0.1;
cube_and_tower(3).pos(2,4) = cube_and_tower(3).pos(2,4) - 0.1;
cube_and_tower(3).pos(3,6) = cube_and_tower(3).pos(3,6) + 0.8;
cube_and_tower(3).pos(1,6) = cube_and_tower(3).pos(1,2) + 0.4;
cube_and_tower(3).pos(2,6) = cube_and_tower(3).pos(2,2) - 0.3;
cube_and_tower(3).pos(:,2) = mean(cube_and_tower(3).pos(:,[4 6 8]), 2);
cube_and_tower(3).pos(3,6) = cube_and_tower(3).pos(3,6) + 1;
if check_constraints_truss(cube_and_tower(3), cube_and_tower(3).pos)
    plot_truss(cube_and_tower(3)); axis tight
%     plot_truss_3D(cube_and_tower(3)); axis tight
    view(cube_and_tower_view); title('cube and tower, step 3')
%     xlabel('x'); ylabel('y'); zlabel('z');
end

cube_and_tower(4) = cube_and_tower(3);
cube_and_tower(4).pos(1:2,7) = cube_and_tower(4).pos(1:2,9);
cube_and_tower(4).pos(3,7) = cube_and_tower(4).pos(3,9) + 2;
cube_and_tower(4).pos(1:2,4) = cube_and_tower(4).pos(1:2,1);
cube_and_tower(4).pos(3,4) = cube_and_tower(4).pos(3,1) + 2;
cube_and_tower(4).pos(1:2,6) = cube_and_tower(4).pos(1:2,8);
cube_and_tower(4).pos(3,6) = cube_and_tower(4).pos(3,8) + 2;
cube_and_tower(4).pos(1:2,2) = cube_and_tower(4).pos(1:2,7);
cube_and_tower(4).pos(3,2) = cube_and_tower(4).pos(3,7) + 2;
if check_constraints_truss(cube_and_tower(4), cube_and_tower(4).pos)
    plot_truss(cube_and_tower(4)); axis tight
%     plot_truss_3D(cube_and_tower(4)); axis tight
    view(cube_and_tower_view); title('cube and tower, step 4')
end
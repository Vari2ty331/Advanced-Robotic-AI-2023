function n = cube26

n.pos =    [2.2684    3.4973    0.1463
            1.3472    3.0688    0.2398
            0.7763    3.5356    0.0108
            2.2345    2.5557    0.1413
            2.1125    3.0335    0.4077
            0.7436    2.6332    0.0111
            1.7575    3.0431    0.0000
            1.5983    3.0566    1.1507
            1.6614    3.0500    0.6501];
% 
% n.pos(:,1) = [1 0 1];
% n.pos(:,2) = [0 0 0];
% n.pos(:,3) = [0 0 1];
% n.pos(:,4) = [1 1 0];
% n.pos(:,5) = [1 1 1];
% n.pos(:,6) = [0 1 0];
% n.pos(:,7) = [0 1 1];
% n.pos(:,8) = [1 0 0];
% n.pos(:,9) = [0.5 0.5 0.5];

% n.pos = n.pos';

n.elist(1,:) = [8 4];
n.elist(2,:) = [2 6];
n.elist(3,:) = [1 5];
n.elist(4,:) = [3 7];
n.elist(5,:) = [8 1];
n.elist(6,:) = [2 3];
n.elist(7,:) = [4 5];
n.elist(8,:) = [6 7];
n.elist(9,:) = [8 2];
n.elist(10,:) = [4 6];
n.elist(11,:) = [1 3];
n.elist(12,:) = [5 7];
n.elist(13,:) = [8 3];
n.elist(14,:) = [3 5];
n.elist(15,:) = [5 6];
n.elist(16,:) = [8 6];
n.elist(17,:) = [8 5];
n.elist(18,:) = [3 6];
n.elist(19,:) = [8 9];
n.elist(20,:) = [1 9];
n.elist(21,:) = [2 9];
n.elist(22,:) = [3 9];
n.elist(23,:) = [4 9];
n.elist(24,:) = [5 9];
n.elist(25,:) = [6 9];
n.elist(26,:) = [7 9];


% top_add_variable;

end
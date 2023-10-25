function n = tower_eugene

n.pos(:,1) = [0 0 0];
n.pos(:,2) = [1 1 0];
n.pos(:,3) = [0 1 0];
n.pos(:,4) = [0 0 1];
n.pos(:,5) = [1 1 1];
n.pos(:,6) = [0 1 1];
n.pos(:,7) = [0 0 2];
n.pos(:,8) = [1 1 2];
n.pos(:,9) = [0 1 2];

n.pos = n.pos';

n.elist = [1 2 ; 2 3 ; 3 1 ; 4 5 ; 5 6 ; 6 4 ; 7 8 ; 8 9 ; 9 7 ; 1 4 ; 2 5 ; 3 6 ; 4 7 ; 5 8 ; 6 9 ]; % 15 members (horizontal, vertical)
n.elist = [n.elist ; 1 6 ; 2 6 ; 4 8]; % diagonal members
    
% top_add_variable;

end
function n = cube_eugene

n.pos(:,1) = [0 0 0];
n.pos(:,2) = [1 0 0];
n.pos(:,3) = [1 1 0];
n.pos(:,4) = [0 1 0];
n.pos(:,5) = [0 0 1];
n.pos(:,6) = [1 0 1];
n.pos(:,7) = [1 1 1];
n.pos(:,8) = [0 1 1];
n.pos(:,9) = [0.5 0.5 0.5];

n.pos = n.pos';

n.elist = [1 2 ; 2 3 ; 3 4 ; 4 1 ; 5 6 ; 6 7 ; 7 8 ; 8 5 ; 1 5 ; 2 6 ; 3 7 ; 4 8]; % 12 members (horizontal, vertical)
n.elist = [n.elist ; 1 8 ; 1 3 ; 2 5]; % diagonal members
n.elist = [n.elist ; 6 9 ; 7 9 ; 8 9]; % members at center node
    
% top_add_variable;

end
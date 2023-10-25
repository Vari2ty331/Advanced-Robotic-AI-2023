function n = cube_URAI_modified

n.pos(1,:) = [0 0 0];
n.pos(2,:) = [1 0 0];
n.pos(3,:) = [1 1 0];
n.pos(4,:) = [0 1 0];
n.pos(5,:) = [0 0 1];
n.pos(6,:) = [1 0 1];
n.pos(7,:) = [1 1 1];
n.pos(8,:) = [0 1 1];
n.pos(9,:) = [0.5 0.5 0.5];

n.elist = [1 2 ; 2 3 ; 3 4 ; 4 1 ; 5 6 ; 6 7 ; 7 8 ; 8 5];
n.elist = [n.elist ; 1 5 ; 2 6 ; 3 7 ; 4 8];
n.elist = [n.elist ; 1 9 ; 2 9 ; 5 9];
n.elist = [n.elist ; 2 7 ; 6 8 ; 4 7];
n.elist = [n.elist ; 3 9 ; 4 9 ; 6 9 ; 7 9 ; 8 9]; % auxilary member

n.member_end = [4 3 ; 3 4 ; 3 4 ; 1 4 ; 4 3 ; 3 4 ; 3 4 ; 4 3];
n.member_end = [n.member_end ; 3 4 ; 3 4 ; 3 4 ; 4 3 ];
n.member_end = [n.member_end ; 3 4 ; 4 3 ; 3 4];
n.member_end = [n.member_end ; 4 1 ; 4 1 ; 3 4];
n.member_end = [n.member_end ; 5 5 ; 5 5 ; 5 5 ; 5 5 ; 5 5]; % auxilary member

end
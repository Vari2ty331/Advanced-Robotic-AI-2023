function n = tower

n.pos(:,1) = [0 -0.5 0];
n.pos(:,2) = [0 0.5 0];
n.pos(:,3) = [sqrt(3)/2 0 0];
n.pos(:,4) = [0 -0.5 1];
n.pos(:,5) = [0 0.5 1];
n.pos(:,6) = [sqrt(3)/2 0 1];
n.pos(:,7) = [0 -0.5 2];
n.pos(:,8) = [0 0.5 2];
n.pos(:,9) = [sqrt(3)/2 0 2];
n.pos(:,10) = [0 -0.5 3];
n.pos(:,11) = [0 0.5 3];
n.pos(:,12) = [sqrt(3)/2 0 3];

n.pos = n.pos';
n.elist = [1 2 ; 2 3 ; 1 3 ; 4 5 ; 5 6 ; 6 4 ; 7 8 ; 8 9 ; 9 7 ; 10 11 ; 11 12 ; 12 10];
n.elist = [n.elist ; 1 4 ; 4 7 ; 7 10 ; 2 5 ; 5 8 ; 8 11 ; 3 6 ; 6 9 ; 9 12];
n.elist = [n.elist ; 1 5 ; 2 6 ; 1 6 ; 4 8 ; 5 9 ; 4 9 ; 7 11 ; 8 12 ; 7 12]; 

end
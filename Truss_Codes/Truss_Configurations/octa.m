function n = octa

n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,2) = [sqrt(3)/6 0.5 0];
n.pos(:,3) = [sqrt(3)/6 -0.5 0];
n.pos(:,4) = [sqrt(3)/3 0 sqrt(6)/3];
n.pos(:,5) = [-sqrt(3)/6 -0.5 sqrt(6)/3];
n.pos(:,6) = [-sqrt(3)/6 0.5 sqrt(6)/3];
n.pos(:,7) = [0 0 sqrt(6)/6];

n.pos = n.pos';

n.elist = [1 2 ; 2 3 ; 1 3 ; 4 5 ; 5 6 ; 4 6 ; 1 5 ; 1 6 ; 2 4 ; 2 6 ; 3 4 ; 3 5 ; 1 7 ; 2 7 ; 3 7 ; 4 7 ; 5 7 ; 6 7];

end
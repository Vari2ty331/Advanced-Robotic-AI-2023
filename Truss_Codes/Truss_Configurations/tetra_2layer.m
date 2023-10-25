
function n = tetra_2layer

addpath(genpath(pwd)); 

h = 0;

n.pos(1,:) = [0 0 0]';
n.pos(2,:) = [sqrt(3)/2 -0.5 0]';
n.pos(3,:) = [sqrt(3)/2 0.5 0]';

n.pos(10,:) = (n.pos(1,:) + n.pos(2,:) + n.pos(3,:))/3 + [0 0 sqrt(6)/3];

n.pos(7,:) = (n.pos(1,:) + n.pos(10,:))/2;
n.pos(8,:) = (n.pos(2,:) + n.pos(10,:))/2;
n.pos(9,:) = (n.pos(3,:) + n.pos(10,:))/2;

n1 = ( (1-h)*n.pos(1,:) + h*n.pos(7,:) );
n2 = ( (1-h)*n.pos(2,:) + h*n.pos(8,:) );
n3 = ( (1-h)*n.pos(3,:) + h*n.pos(9,:) );

n.pos(4,:) = (n1 + n2)/2;
n.pos(5,:) = (n2 + n3)/2;
n.pos(6,:) = (n1 + n3)/2;

n.elist = [1 4; 1 6 ; 1 7; 2 4 ; 2 5 ; 2 8 ; 3 5 ; 3 6 ; 3 9 ; 4 5 ; 4 6 ; 4 7 ; 4 8 ; 5 6 ; 5 8 ; 5 9 ; 6 7 ; 6 9 ; 7 8 ; 7 9 ; 7 10 ; 8 9 ; 8 10 ; 9 10];

end


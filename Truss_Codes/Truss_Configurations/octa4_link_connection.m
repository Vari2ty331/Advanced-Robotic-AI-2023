function n = octa12_link_connection

n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,2) = [sqrt(3)/6 0.5 0];
n.pos(:,3) = [sqrt(3)/6 -0.5 0];
n.pos(:,4) = [sqrt(3)/3 0 sqrt(6)/3];
n.pos(:,5) = [-sqrt(3)/6 -0.5 sqrt(6)/3];
n.pos(:,6) = [-sqrt(3)/6 0.5 sqrt(6)/3];

n.pos = n.pos';

n.elist = [1 2 ; 2 3 ; 3 1 ; 4 5 ; 5 6 ; 6 4 ; 1 5 ; 6 1 ; 4 2 ; 2 6 ; 3 4 ; 5 3];
n.link_connection{1} = [1 3 ; 3 7 ; 7 8 ; 8 1];
n.link_connection{2} = [1 2 ; 2 9 ; 9 10 ; 10 1];
n.link_connection{3} = [2 3 ; 3 12 ; 12 11 ; 11 2];
n.link_connection{4} = [4 6 ; 6 9 ; 9 11 ; 11 4];
n.link_connection{5} = [4 5 ; 5 7 ; 7 12 ; 12 4];  
n.link_connection{6} = [5 6 ; 6 10 ; 10 8 ; 8 5];

end
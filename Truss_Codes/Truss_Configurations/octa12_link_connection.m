function n = octa12_link_connection

% Bottom nodes
n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,2) = [sqrt(3)/6 0.5 0];
n.pos(:,3) = [sqrt(3)/6 -0.5 0];

% Top nodes
n.pos(:,4) = [sqrt(3)/3 0 sqrt(6)/3];
n.pos(:,5) = [-sqrt(3)/6 -0.5 sqrt(6)/3];
n.pos(:,6) = [-sqrt(3)/6 0.5 sqrt(6)/3];

n.pos = n.pos';

% Bottom members
n.elist(1,:) = [1 2];
n.elist(2,:) = [2 3];
n.elist(3,:) = [3 1];

% Top members
n.elist(4,:) = [4 5];
n.elist(5,:) = [5 6];
n.elist(6,:) = [6 4];

% Side members
n.elist(7,:) = [1 5];
n.elist(8,:) = [6 1];
n.elist(9,:) = [4 2];
n.elist(10,:) = [2 6];
n.elist(11,:) = [3 4];
n.elist(12,:) = [5 3];

n.link_connection{1} = [1 3 ; 3 7 ; 7 8 ; 8 1];
n.link_connection{2} = [1 2 ; 2 9 ; 9 10 ; 10 1];
n.link_connection{3} = [2 3 ; 3 12 ; 12 11 ; 11 2];
n.link_connection{4} = [4 6 ; 6 9 ; 9 11 ; 11 4];
n.link_connection{5} = [4 5 ; 5 7 ; 7 12 ; 12 4];  
n.link_connection{6} = [5 6 ; 6 10 ; 10 8 ; 8 5];

end
function n = octa12_200204_test

% Bottom nodes
n.pos(:,1) = [-sqrt(3)/3 0 0];
n.pos(:,6) = [sqrt(3)/6 0.5 0];
n.pos(:,4) = [sqrt(3)/6 -0.5 0];

% Top nodes
n.pos(:,3) = [sqrt(3)/3 0 sqrt(6)/3];
n.pos(:,2) = [-sqrt(3)/6 -0.5 sqrt(6)/3];
n.pos(:,5) = [-sqrt(3)/6 0.5 sqrt(6)/3];

n.pos = n.pos';

% Bottom members
n.elist(5,:) = [6 1];
n.elist(6,:) = [4 6];
n.elist(11,:) = [1 4];

% Top members
n.elist(1,:) = [2 3];
n.elist(2,:) = [5 2];
n.elist(12,:) = [3 5];

% Side members
n.elist(3,:) = [6 3];
n.elist(4,:) = [3 4];
n.elist(7,:) = [4 2];
n.elist(8,:) = [2 1];
n.elist(9,:) = [1 5];
n.elist(10,:) = [5 6];

n.link_connection{1} = [5 9 ; 9 8 ; 8 11 ; 11 5];
n.link_connection{2} = [1 2 ; 2 8 ; 8 7 ; 7 1];
n.link_connection{3} = [1 12 ; 12 3 ; 3 4 ; 4 1];
n.link_connection{4} = [4 7 ; 7 11 ; 11 6 ; 6 4];
n.link_connection{5} = [2 12 ; 12 10 ; 10 9 ; 9 2];  
n.link_connection{6} = [3 10 ; 10 5 ; 5 6 ; 6 3];

end
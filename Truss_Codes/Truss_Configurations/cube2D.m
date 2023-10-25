function n = cube2D
% cube with 9 nodes

n.pos(:,1) = [ 0 0 0 ];
n.pos(:,2) = [ 1 0 0 ];
n.pos(:,3) = [ 1 1 0 ];
n.pos(:,4) = [ 0 1 0 ];
n.pos(:,5) = [ 0.5 0.5 0 ];

n.pos = n.pos';

n.elist = [1,2;2,3;3,4;1,4;1,5;2,5;3,5;4,5];

end

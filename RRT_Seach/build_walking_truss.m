%% walking?
close all;
truss_walk(1) = truss_intermed;
truss_walk(1).elist = tall300.elist;
truss_walk(1).edge_adj = tall300.edge_adj;
truss_walk(1).adj = tall300.adj;
truss_walk(1).pos(:,9) = truss_walk(1).pos(:,6);
truss_walk(1).pos(3,9) = truss_walk(1).pos(3,9) - 0.2;
truss_walk(1).pos(:,1) = [-1 1 -1]; % extra ground point

if check_constraints_truss(truss_walk(1), truss_walk(1).pos)
    plot_truss(truss_walk(1)); axis tight; title('Toward walk step 1')
    view([70 20]);
end
% if ~check_com(truss_walk(1),truss_walk(1).pos)
%     disp('Warning');
% end


truss_walk(2) = truss_walk(1);
truss_walk(2).pos(:,4) = [0 1 0];
truss_walk(2).pos(:,5) = [-1 -0.5 0];
truss_walk(2).pos(3,9) = -1;
truss_walk(2).pos(:,1) = [-1 1 -1];

if check_constraints_truss(truss_walk(2), truss_walk(2).pos)
    plot_truss(truss_walk(2)); axis tight
    view([70, 20]); title('Toward walk step 2')
end
% if ~check_com(truss_walk(2),truss_walk(2).pos)
%     disp('Warning');
% end


truss_walk(3) = truss_walk(2);

truss_walk(3).pos(2,5) = 0;
truss_walk(3).pos([1 2],3) = [-0.5 -1];
truss_walk(3).pos(:,4)=truss_walk(3).pos(:,7)+[0;0.2;0.2];
truss_walk(3).pos([1 2],2) = [-0.5 1];

if check_constraints_truss(truss_walk(3), truss_walk(3).pos)
    plot_truss(truss_walk(3)); axis tight
    view([70, 20]); title('Toward walk step 3')
end
% if ~check_com(truss_walk(3),truss_walk(3).pos)
%     disp('Warning');
% end


truss_walk(4) = truss_walk(3);
truss_walk(4).pos(2,5)=0.5;
truss_walk(4).pos(:,1)=mean(truss_walk(4).pos(:,[4 5]),2);
truss_walk(4).pos(:,6)=truss_walk(4).pos(:,8)+[0.3;0;0.5];
truss_walk(4).pos(:,7)=[0.4;0;-1];


if check_constraints_truss(truss_walk(4), truss_walk(4).pos)
    plot_truss(truss_walk(4)); axis tight
    view([70, 20]); title('Toward walk step 4')
end
% if ~check_com(truss_walk(4),truss_walk(4).pos)
%     disp('Warning');
% end


truss_walk(5) = truss_walk(4);
truss_walk(5).pos(:,4)=[0;1;-1];
truss_walk(5).pos(:,9)=[-0.7;0;-0.2];
truss_walk(5).pos(1,[8 6])=truss_walk(5).pos(1,[8 6])+[0.5 0.5];
truss_walk(5).pos(1,1)=-0.2;


if check_constraints_truss(truss_walk(5), truss_walk(5).pos)
    plot_truss(truss_walk(5)); axis tight
    view([70, 20]); title('Toward walk step 5')
end
% if ~check_com(truss_walk(5),truss_walk(5).pos)
%     disp('Warning');
% end


truss_walk(6) = truss_walk(5);
truss_walk(6).pos(:,5)=truss_walk(6).pos(:,1);
truss_walk(6).pos(1,5)=0;
% truss_walk(6).pos(:,1)=[0.8;0.8;-1];
truss_walk(6).pos(:,1)=[1 0.7 -1];
truss_walk(6).pos(1,9)=-0.2;
truss_walk(6).pos(:,3)=truss_walk(6).pos(:,9)+[0;0;0.5];


if check_constraints_truss(truss_walk(6), truss_walk(6).pos)
    plot_truss(truss_walk(6)); axis tight
    view([70, 20]); title('Toward walk step 6')
end
% if ~check_com(truss_walk(6),truss_walk(6).pos)
%     disp('Warning');
% end




truss_walk(7) = truss_walk(6);
truss_walk(7).pos(:,5)=[1;0.2;0];
truss_walk(7).pos(:,3)=mean(truss_walk(7).pos(:,[1 7 8]),2)+[0;-0.2;0.5];
truss_walk(7).pos(:,9)=mean(truss_walk(7).pos(:,[1 7 8]),2);
truss_walk(7).pos(:,9)=[1;-0.7;-1];


if check_constraints_truss(truss_walk(7), truss_walk(7).pos)
    plot_truss(truss_walk(7)); axis tight
    view([70, 20]); title('Toward walk step 7')
end
% if ~check_com(truss_walk(7),truss_walk(7).pos)
%     disp('Warning');
% end


truss_walk(8) = truss_walk(7);
truss_walk(8).pos(:,7)=[0;0;-0.3];
truss_walk(8).pos([1 2],8)=truss_walk(8).pos([1 2],7);
truss_walk(8).pos([1 2],6)=truss_walk(8).pos([1 2],7);
truss_walk(8).pos(:,3)=[-1;-0.7;-1];
% truss_walk(8).pos(:,4)=[0;0.7;0.3];
% truss_walk(8).pos([1 2],2) = [-1 0.7];
truss_walk(8).pos([1 2],5)=truss_walk(8).pos([1 2],9);


if check_constraints_truss(truss_walk(8), truss_walk(8).pos)
    plot_truss(truss_walk(8)); axis tight
    view([70, 20]); title('Toward walk step 8')
end
% if ~check_com(truss_walk(8),truss_walk(8).pos)
%     disp('Warning');
% end


truss_walk(9) = truss_walk(8);
truss_walk(9).pos(:,4)=[0;0.7;0.3];%
truss_walk(9).pos([1 2],2) = [-1 0.7];%
truss_walk(9).elist=walker301.elist;
truss_walk(9).edge_adj=walker301.edge_adj;
truss_walk(9).adj=walker301.adj;
truss_walk(9).pos(3,8)=1.5;
truss_walk(9).pos(3,7)=-0.3;
% truss_walk(9).pos([1 2],1)=[1 0.7];
truss_walk(9).pos(3,5)=0.8;
truss_walk(9).pos(:,6)=[-0.9;0;0.4];
truss_walk(9).pos(3,4)=0.9;



if check_constraints_truss(truss_walk(9), truss_walk(9).pos)
    plot_truss(truss_walk(9)); axis tight
    view([70, 20]); title('Toward walk step 9')
end
% if ~check_com(truss_walk(9),truss_walk(9).pos)
%     disp('Warning');
% end

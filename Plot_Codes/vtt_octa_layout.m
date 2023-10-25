close all

n = octa12_link_connection;

initial_plot

n.end(1,:) = [0 1];
n.end(2,:) = [0 1];
n.end(3,:) = [0 1];
n.end(4,:) = [0 1];
n.end(5,:) = [0 1];
n.end(6,:) = [0 1];
n.end(7,:) = [0 1];
n.end(8,:) = [0 1];
n.end(9,:) = [0 1];
n.end(10,:) = [0 1];
n.end(11,:) = [0 1];
n.end(12,:) = [0 1];

hold on 

L_end = 0.1;
M_size = 30;

for i = 1:length(n.elist)
    P1 = n.pos(n.elist(i,1),:);
    P2 = n.pos(n.elist(i,2),:);
    v = (P2 - P1) / norm(P2 - P1);
    P_end1 = P1 + L_end*v;
    P_end2 = P2 - L_end*v;
        
    P_end = P_end1;
    if n.end(i,1) == 0
        plot3(P_end(1),P_end(2),P_end(3),'b.','markersize',M_size);
    else
        plot3(P_end(1),P_end(2),P_end(3),'r.','markersize',M_size);
    end
    
    P_end = P_end2;
    if n.end(i,2) == 0
        plot3(P_end(1),P_end(2),P_end(3),'b.','markersize',M_size);
    else
        plot3(P_end(1),P_end(2),P_end(3),'r.','markersize',M_size);
    end
end


axis equal
axis ([-1 1 -1 1 0 1.5])
hold off
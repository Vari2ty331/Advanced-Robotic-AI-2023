% Given initial configuration n, plot

%% Plot setting
drive_plot = 1;
linkage_plot = 1;

%% Initial value setting

pos = data.n_ini.pos;
elist = data.n_ini.elist;

n = data.n_ini;
% n = octa12;
% initial variables
x_ini = reshape(pos,[numel(pos),1]);
L_ini = strut_length_cal(n);

x = x_ini;
L = L_ini;

% Center of mass matrix
if exist('data.M')
    M = data.M;
else
    M = [ones(1,length(pos))  zeros(1,length(pos))  zeros(1,length(pos))
         zeros(1,length(pos)) ones(1,length(pos))   zeros(1,length(pos))
         zeros(1,length(pos)) zeros(1,length(pos))  zeros(1,length(pos))]/length(pos);
end


%% Initial configuration plot
figure

hold on

for i = 1:length(elist)

    a = elist(i,1);
    b = elist(i,2);
    
    Xa = pos(a,1);
    Ya = pos(a,2);
    Za = pos(a,3);
    Xb = pos(b,1);
    Yb = pos(b,2);
    Zb = pos(b,3);
    
    plot3([Xa Xb],[Ya Yb],[Za Zb],'LineWidth',1.5,'Color','k')
    view(3)
end

plot3(pos(:,1),pos(:,2),pos(:,3),'k.','MarkerSize',40,'Color','b')
x_cm = M*x; % center of mass
% plot3(x_cm(1),x_cm(2),0,'m.','MarkerSize',40)


for i = 1:length(pos)
    text( pos(i,1)-0.03, pos(i,2)+0.1, pos(i,3) + 0.15, num2str(i),'Color','k','FontSize',15,'FontName','Times New Roman' )
end

for i = 1:length(elist)
    a = elist(i,1);
    b = elist(i,2);
%     text( (pos(a,1) + pos(b,1))/2, (pos(a,2) + pos(b,2))/2, (pos(a,3) + pos(b,3))/2, num2str(i), 'color','r','FontSize',13)
end

p = fill3(pos(1:3,1),pos(1:3,2),pos(1:3,3),[254, 204, 203]./255,'LineStyle','none');
% p.FaceAlpha = 0.5;  

% Friction drive plot
% if drive_plot
%     drive_com = 0.2;
%     for i = 1:length(elist)    
%         u = ( pos(elist(i,1),:) - pos(elist(i,2),:) ) / L(i);
%         for j = 1:length(com_member{i})
%             drive_pos = pos(elist(i,2),:) + u*drive_com;
%             plot3(drive_pos(1),drive_pos(2),drive_pos(3),'r.','Markersize',30);
%         end
%     end
% end

% % Linkage plot
% if linkage_plot
%     link_connection = n.link_connection;
%     linkage_com = 0.15;
%     for i = 1:length(link_connection)
%         for j = 1:size(link_connection{i},1)
%             edge1 = elist( link_connection{i}(j,1), :);
%             edge2 = elist( link_connection{i}(j,2), :);
%             
%             u1 = pos(edge1(1),:) - pos(i,:);
%             if norm(u1) == 0
%                 u1 = pos(edge1(2),:) - pos(i,:);
%             end
%             
%             u2 = pos(edge2(1),:) - pos(i,:);
%             if norm(u2) == 0
%                 u2 = pos(edge2(2),:) - pos(i,:);
%             end
%             
%             linkage_pos(1,:) = pos(i,:) + u1*linkage_com;
%             linkage_pos(2,:) = pos(i,:) + u2*linkage_com;
%             plot3(linkage_pos(:,1),linkage_pos(:,2),linkage_pos(:,3),'k');
%         end
%     end
% end

xlim([-1 2])
ylim([-1 2])
zlim([0 3])
% grid on
axis equal
axis off

set(gcf, 'color', 'none');    
set(gca, 'color', 'none');

hold off
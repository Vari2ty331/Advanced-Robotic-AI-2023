dt = data.dt;
n = data.n_ini;
clear MM

%% data plot
figure

% Length plot
subplot(3,2,1)
plot(dt:dt:size(data.x,1)*dt,data.L_max,'r');
hold on
plot(dt:dt:size(data.x,1)*dt,data.L_min,'b');
plot([0 size(data.x,1)*dt],[data.desired_L_min(1) data.desired_L_min(1)],':k','linewidth',1.5);
plot([0 size(data.x,1)*dt],[data.desired_L_max(1) data.desired_L_max(1)],':k','linewidth',1.5);
plot([0 size(data.x,1)*dt],[1.8865 1.8865],':','color',[0.7 0.7 0.7],'linewidth',1.5); % max. length in case of comp.75, tens.70, stb.0.07
hold off
% xlim([0 (size(data.x,1)-1)*dt])
xlim([0 size(data.x,1)*dt])
ylim([data.desired_L_min(1)-0.2 data.desired_L_max(1)+0.2])

legend('max length','min length','location','EastOutSide')
xlabel('t(sec)')
ylabel('length')
title('Length constraint (L_{long})','fontsize',13)

% Angle plot
subplot(3,2,2)

plot(dt:dt:size(data.x,1)*dt,data.angle_min/pi*180,'b');
hold on
plot(dt:dt:size(data.x,1)*dt,data.angle_max/pi*180,'r');
plot([0 size(data.x,1)*dt],[data.desired_angle_min data.desired_angle_min]*180/pi,':k','linewidth',1.5);
plot([0 size(data.x,1)*dt],[data.desired_angle_max data.desired_angle_max]*180/pi,':k','linewidth',1.5);
hold off
xlim([0 size(data.x,1)*dt])
ylim([0 180])
legend('min angle','max angle','location','EastOutSide')
xlabel('t(sec)')
ylabel('angle(deg)')
title('Angle constraint','fontsize',13)

% Dihedral angle plot
subplot(3,2,3)
plot(dt:dt:size(data.x,1)*dt,data.dihedral_angle_min/pi*180,'b');
hold on
plot([0 size(data.x,1)*dt],[data.desired_dihedral_angle_min data.desired_dihedral_angle_min]*180/pi,':k','linewidth',1.5);
hold off
xlim([0 size(data.x,1)*dt])
ylim([20 120])
legend('min dihedral angle','location','EastOutSide')
% legend('max angle','min angle','location','EastOutSide')
xlabel('t(sec)')
ylabel('angle(deg)')
title('Dihedral angle constraint','fontsize',13)

% Collision plot
subplot(3,2,4)
plot(dt:dt:size(data.d_min,1)*dt,data.d_min,'r');
hold on
plot([0 size(data.x,1)*dt],[data.desired_d_min data.desired_d_min],':k','linewidth',1.5);
hold off
xlim([0 size(data.x,1)*dt])
ylim([0 1.2])

legend('min distance','location','EastOutSide')
xlabel('t(sec)')
ylabel('distance')
title('Collision constraint','fontsize',13)

% Force graph
subplot(3,2,5)
plot(max(data.compressive_force'),'r')
hold on
plot(-max(data.tensile_force'),'b')
plot([1 length(data.x)],data.max_compressive_force*ones(1,2),'r:','LineWidth',2)
plot([1 length(data.x)],-data.max_tensile_force*ones(1,2),'b:','LineWidth',2)
legend('compressive','tensile','location','eastoutside')
xlabel('time (sec)')
ylabel('force (N)')
xlim([0 size(data.x,1)*dt])
hold off

set(gcf,'units', 'pixels', 'pos',[0 0 1400 700])   
%% Motion plot
figure

clear MM        
play_speed = 1;
M_index = 1;

for kk = 1:(size((data.x),1))*play_speed
    
    k = ceil(kk/play_speed);
    
   %% plot setting
    grid on
    set(gcf,'units', 'pixels', 'pos',[100 100 500 500])   
 
    %% Force plot
    
    fixed_node = data.fixed_node{k};
    
    n.pos = reshape(data.x(k,:),size(n.pos));
    
    fixed_nodes_surface_normal = cross(n.pos(fixed_node(1),:)-n.pos(fixed_node(2),:),n.pos(fixed_node(2),:)-n.pos(fixed_node(3),:)) / norm(cross(n.pos(fixed_node(1),:)-n.pos(fixed_node(2),:),n.pos(fixed_node(2),:)-n.pos(fixed_node(3),:)));
    fixed_nodes_surface_normal = fixed_nodes_surface_normal.';
    Plane_A = fixed_nodes_surface_normal(1);
    Plane_B = fixed_nodes_surface_normal(2);
    Plane_C = fixed_nodes_surface_normal(3);
    Plane_D = dot(fixed_nodes_surface_normal,n.pos(fixed_node(1),:));          % Ax + By + Cz = D (Plane equation of inclined surface)
    
    plot3(data.x_cm(k,1),data.x_cm(k,2),(Plane_D - Plane_A*data.x_cm(k,1) - Plane_B*data.x_cm(k,2))/Plane_C,'m.','MarkerSize',20)
    
    hold on

    for i = 1:length(n.pos)
        text( n.pos(i,1) + 0.05, n.pos(i,2) + 0.05, n.pos(i,3) + 0.05, num2str(i) )
    end
    
    temp = data.fixed_node{k,1};
    sup_polygon = temp( convhull(n.pos(temp,1),n.pos(temp,2)) );
    
    if length(sup_polygon) > 2
        patch( n.pos(sup_polygon,1), n.pos(sup_polygon,2), n.pos(sup_polygon,3,1),[1,0.8,0.8]);
    else
        plot3( n.pos(sup_polygon,1), n.pos(sup_polygon,2), n.pos(sup_polygon,3),'r');
    end
    
    % Support polygon plot
    

    
    
    % edge plot
    for i = 1:length(n.elist)

        a = n.elist(i,1);
        b = n.elist(i,2);

        Xa = n.pos(a,1);
        Ya = n.pos(a,2);
        Za = n.pos(a,3);
        Xb = n.pos(b,1);
        Yb = n.pos(b,2);
        Zb = n.pos(b,3);
        
        line_width = 10
        
%         if isempty(find(data.fixed_member{k} == i, 1))
%             line_width = 5;
%         else
%             line_width = 10;
%         end
        
        if data.tensile_force(k,i) ~= 0
            edge_force = round(data.tensile_force(k,i));
            max_edge_force = data.max_tensile_force;
            C = winter;
            
            if edge_force == 0
                CC = C(64,:);
            elseif edge_force >= max_edge_force
                CC = C(1,:);
            else
                CC = C(64 - floor((edge_force/max_edge_force)^2*64),:); % select color from color map
            end    
            
           text((Xa+Xb)/2+0.05, (Ya+Yb)/2+0.05, (Za+Zb)/2+0.05,num2str(edge_force),'Color','b');
            plot3([Xa Xb],[Ya Yb],[Za Zb],'LineWidth',line_width,'Color',CC)

        else
            edge_force = round(data.compressive_force(k,i));
            max_edge_force = data.max_compressive_force;
            C = autumn;
            
            if edge_force == 0
                CC = C(64,:);
            elseif edge_force >= max_edge_force
                CC = C(1,:);
            else
                CC = C(64 - floor((edge_force/max_edge_force)^2*64),:); % select color from color map
            end    
            
           text((Xa+Xb)/2+0.05, (Ya+Yb)/2+0.05, (Za+Zb)/2+0.05,num2str(edge_force),'Color','r');
            plot3([Xa Xb],[Ya Yb],[Za Zb],'LineWidth',line_width,'Color',CC)
        end

    end
         
    plot3(n.pos(:,1),n.pos(:,2),n.pos(:,3),'k.','MarkerSize',30)
    
    % Reaction plot
%     
%     surface_normal_set = reshape(data.normal_vector{k},[3, length(fixed_node)]);
%     
%     for Gravity_vector_index = 1 : length(fixed_node)
%         n_hat(:,Gravity_vector_index) = surface_normal_set(:,Gravity_vector_index);
%         Gravity_Direction(:,Gravity_vector_index) = [0;0;-1;] - (dot([0;0;-1;],n_hat(:,Gravity_vector_index)))*n_hat(:,Gravity_vector_index);
%     
%         if norm(Gravity_Direction(:,Gravity_vector_index)) <= 1e-5
%             Gravity_Direction(:,Gravity_vector_index) = [0;0;0;];
%         else
%             Gravity_Direction(:,Gravity_vector_index) = Gravity_Direction(:,Gravity_vector_index)/norm(Gravity_Direction(:,Gravity_vector_index));      % Gravity direction projected on the surface
%         end
%     end
    
    Normal = reshape(data.Reaction_Normal_Force{k},3,length(fixed_node));
    Normal = Normal.';
    Req_Fric = reshape(data.Reaction_Req_Fric{k},3,length(fixed_node));
    Req_Fric = Req_Fric.';

    for f_num = 1 : length(fixed_node)
%         quiver3(n.pos(data.fixed_node{k}(f_num),1), n.pos(data.fixed_node{k}(f_num),2), n.pos(data.fixed_node{k}(f_num),3), data.Reaction(k,3*f_num - 2), data.Reaction(k,3*f_num - 1), data.Reaction(k,3*f_num), 0.01, 'Color','k','LineWidth', 3)
        quiver3(n.pos(data.fixed_node{k}(f_num),1), n.pos(data.fixed_node{k}(f_num),2), n.pos(data.fixed_node{k}(f_num),3), Normal(f_num,1), Normal(f_num,2), Normal(f_num,3), 0.005, 'Color','k','LineWidth', 3)
        if norm(Req_Fric(f_num,:)) > data.MaxStaticCoeff * norm(Normal(f_num,:)) + 10^-2
            quiver3(n.pos(data.fixed_node{k}(f_num),1), n.pos(data.fixed_node{k}(f_num),2), n.pos(data.fixed_node{k}(f_num),3), Req_Fric(f_num,1), Req_Fric(f_num,2), Req_Fric(f_num,3), 0.005, 'Color','m','LineWidth', 3)
        else
            quiver3(n.pos(data.fixed_node{k}(f_num),1), n.pos(data.fixed_node{k}(f_num),2), n.pos(data.fixed_node{k}(f_num),3), Req_Fric(f_num,1), Req_Fric(f_num,2), Req_Fric(f_num,3), 0.005, 'Color','g','LineWidth', 3)
        end
    end
    
    hold off
    title([num2str((k-1)*dt) ' sec'])
    xlim([data.x_cm(k,1)-1.5 data.x_cm(k,1)+1.5])
    ylim([data.x_cm(k,2)-1.5 data.x_cm(k,2)+1.5])
    zlim([0 3])
    axis equal
    
    MM(M_index) = getframe(gcf);
    M_index = M_index + 1;
   
end % for end

video = VideoWriter('motion_force_plot_old.avi','Motion JPEG AVI');
% video = VideoWriter('motion_force_plot.avi','compressed AVI');
open(video)
writeVideo(video,MM);
close(video)
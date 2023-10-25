dt = data.dt;
n = data.n_ini;

index = 1;

%% Motion plot

Plot_3Dimension = 1;

clear MM        
play_speed = 0.33;
% play_speed = 1;
M_index = 1;
% kk = 113;
fig = figure('Name', 'Locomotion plot');
set(gcf,'color','w');
ax = gca;
for kk = 1:(size((data.x),1)-1)*play_speed
% kk = 357;    
    % Support polygon & obstacle plot

    plot3(0,0,0)
    hold on
    
    camlight

    for k = 1 : length(Ground)
        [~,GroundObj] = show(Ground(k));
        GroundObj.FaceColor = [0.7 0.7 0.7];
        GroundObj.EdgeColor = 'none';
    end

    for i = 1:length(Final_Path_T)
        plot3(Final_Path_T(i).n([1 2 3 1],1),Final_Path_T(i).n([1 2 3 1],2),Final_Path_T(i).n([1 2 3 1],3),'g','linewidth',1)
%         axis(limit);
        axis equal
    end
    
    k = ceil(kk/play_speed);
    n.pos = reshape(data.x(k,:),size(n.pos));
    
    temp = data.fixed_node{k,1};
    sup_polygon = temp( convhull(n.pos(temp,1),n.pos(temp,2)) );
    
    if length(sup_polygon) > 2
        patch( n.pos(sup_polygon,1), n.pos(sup_polygon,2), n.pos(sup_polygon,3) + 0.01,[1,0.8,0.8]);
    else
        plot3( n.pos(sup_polygon,1), n.pos(sup_polygon,2), n.pos(sup_polygon,3) + 0.01,'r');
    end
  
    
    if Plot_3Dimension == 0
        for i = 1:length(n.elist)

            a = n.elist(i,1);
            b = n.elist(i,2);

            Xa = n.pos(a,1);
            Ya = n.pos(a,2);
            Za = n.pos(a,3);
            Xb = n.pos(b,1);
            Yb = n.pos(b,2);
            Zb = n.pos(b,3);

            plot3([Xa Xb],[Ya Yb],[Za Zb],'LineWidth',4)

        end
        % print 2D data
        distance = data.x_cm(k,1) - data.x_cm(1,1);
        if data.rolling(k) == 1
            phase = 'rolling';
            node_type = 'b.';
        elseif data.rolling(k) == 2
            phase = 'transient';
            node_type = 'g.';
        else
            phase = 'control';
            node_type = 'k.';
        end

        plot3(n.pos(:,1),n.pos(:,2),n.pos(:,3),node_type,'MarkerSize',40)
        x_cm = data.x_cm(k,:); % center of mass
    else
        pos = reshape(data.x(k,:),size(n.pos));
        elist = n.elist;

        r_member = 0.04;
        r_node = 0.10;

        first_plot = true;
        truss = n;
        if isfield(truss, 'h') && isvalid(truss.h.fig)
            first_plot = false;
            h = truss.h;
        else
    %         % make figure window
    %         h.fig = figure;
        end

        % plot members as cylinders
        m_p1 = pos(elist(:,1),:)';
        m_p2 = pos(elist(:,2),:)';

        m_x1 = m_p1(1,:);
        m_x2 = m_p2(1,:);

        m_y1 = m_p1(2,:);
        m_y2 = m_p2(2,:);

        m_z1 = m_p1(3,:);
        m_z2 = m_p2(3,:);

        [X_cyl, Y_cyl, Z_cyl] = cylinder(r_member*ones(10,1), 10);
        for ii = length(m_x1):-1:1
            x1 = [m_x1(ii); m_y1(ii); m_z1(ii)];
            x2 = [m_x2(ii); m_y2(ii); m_z2(ii)];
            r3 = x2-x1;
            l = norm(r3);
            r3 = r3/l;
            e1 = [1;0;0];
            e3 = [0;0;1];
            R_axis = cross(e3, r3);
            R_angle = acos(e3'*r3);
            if R_angle < eps || pi - R_angle < eps
                R_axis = e1;
            end
            if first_plot
                h.t(ii) = hgtransform;
            end
            R = makehgtform('axisrotate',R_axis,R_angle);
            S = makehgtform('scale',[1,1,l]);
            T = makehgtform('translate',x1);
            h.t(ii).Matrix = T*R*S;
            if first_plot
                CC = [0.9, 0.9, 0.9];
                surf(X_cyl, Y_cyl, Z_cyl, 'EdgeColor','none', 'facecolor', CC, 'facelighting', 'gouraud', 'Parent', h.t(ii));
            end
        end

        % plot nodes as spheres
        [X_sph, Y_sph, Z_sph] = sphere(12);
        for ii = size(pos,1):-1:1
            if first_plot
                h.tn(ii) = hgtransform;
            end
            h.tn(ii).Matrix = [r_node*eye(3), pos(ii,:)'; 0,0,0,1];
            if first_plot
                surf(X_sph, Y_sph, Z_sph, 'EdgeColor','none', 'facecolor', [0.2, 0.2, 1], 'facelighting', 'gouraud', 'Parent', h.tn(ii));
            end

        end
    end
    
    
    xlim([-2 6])
    ylim([-2 6])
    zlim([0 5])
    
    grid on
    set(gcf,'units', 'pixels', 'pos',[100 100 800 700])
    view(-63,30)
    ax.FontSize = 15;
    ax.FontName = 'Times';
    ax.XTick = [-2 -1 0 1 2 3 4 5 6];
    ax.YTick = [-2 -1 0 1 2 3 4 5 6];
    plot3(param.goal(1),param.goal(2),param.goal(3),'r.','Markersize',30)
    plot3(param.init_com(1),param.init_com(2),param.init_com(3),'g.','Markersize',30)
    hold off
    MM(M_index) = getframe(gcf);
    M_index = M_index + 1;
end

% video = VideoWriter('Motion_Primitive_Random_ShortTravel.avi','Uncompressed AVI');
% open(video)
% writeVideo(video,MM);
% close(video)
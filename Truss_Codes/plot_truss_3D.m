function [ h ] = plot_truss_3D( truss, r_sph, r_cyl, opt_axes )
%plot_truss_3D - This plots a detailed truss model.   
%   If the input truss has never been plotted before, this function will
%   create a new figure for it. In order to inform the program that a
%   truss has been plotted, it needs to have a valid handle field, truss.h
%   This allows a user to call plotting functions in a loop, such as:
%   |    while(cond)
%   |        truss.h = plot_3Dtruss(truss)
%   |    end
%   and the user does not need to worry about special cases for the first
%   time it is plotted, nor about switching figures when multiple
%   independent trusses are plotted

if nargin < 2 || isempty(r)
    r_sph = 0.10; % 0.025 or 0.04
    r_cyl = 0.03;
end

first_plot = true;
if isfield(truss, 'h') && ~isempty(truss.h) && isgraphics(truss.h.ax) && isvalid(truss.h.ax)
    % use the axes from last time
    first_plot = false;
    h = truss.h;
elseif nargin > 3 && ~isempty(opt_axes) && isgraphics(opt_axes) && isvalid(opt_axes)
    % use the user specified axes
    h.ax = opt_axes;
    h.fig = h.ax.Parent;
else
    % make new axes (and a new figure window)
    h.fig = figure;
    h.ax = axes;
end

elist = truss.elist;
pos = truss.pos;

if size(pos,1) > size(pos,2)
    pos = pos';
end

% plot members as cylinders
m_p1 = pos(:,elist(:,1));
m_p2 = pos(:,elist(:,2));

m_x1 = m_p1(1,:);
m_x2 = m_p2(1,:);

m_y1 = m_p1(2,:);
m_y2 = m_p2(2,:);

m_z1 = m_p1(3,:);
m_z2 = m_p2(3,:);

hold(h.ax, 'on')
[X_cyl, Y_cyl, Z_cyl] = cylinder(r_cyl*ones(10,1), 10);
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
        h.t(ii) = hgtransform(h.ax);
    end
    R = makehgtform('axisrotate',R_axis,R_angle);
    S = makehgtform('scale',[1,1,l]);
    T = makehgtform('translate',x1);
    h.t(ii).Matrix = T*R*S;
    if first_plot
        surf(X_cyl, Y_cyl, Z_cyl, 'facecolor', [0.9, 0.9, 0.9], 'EdgeColor',[0.2, 0.2, 0.2], 'facelighting', 'gouraud', 'Parent', h.t(ii));
%         surf(X_cyl, Y_cyl, Z_cyl, 'facecolor', [0.9, 0.9, 0.9], 'facelighting', 'flat', 'Parent', h.t(ii));
    end
end
    
% plot nodes as spheres
[X_sph, Y_sph, Z_sph] = sphere(12);
for ii = size(pos,2):-1:1
    if first_plot
        h.tn(ii) = hgtransform(h.ax);
    end
    h.tn(ii).Matrix = [r_sph*eye(3), pos(:,ii); 0,0,0,1];
    if first_plot
        surf(X_sph, Y_sph, Z_sph, 'facecolor', [0.1, 0.1, 1], 'EdgeColor','none', 'facelighting', 'gouraud', 'Parent', h.tn(ii));
%         surf(X_sph, Y_sph, Z_sph, 'facecolor', [0.2, 0.2, 1], 'facelighting', 'flat', 'Parent', h.tn(ii));
    end
    
end

if first_plot
    axis(h.ax, 'equal')
    axis(h.ax, 'tight')
    axis(h.ax, 'on')
    grid(h.ax, 'on')
end


showLength = false;
showAngle = false;

if showLength
    l = lengths(elist, pos);
    str{1} = ['Min Length: ', num2str(min(l)),'   Max Length: ', num2str(max(l))];
end

if showAngle
    adjm = edgelist_to_adjmatrix(elist);
    [min_a, ~] = min_member_angle(adjm, pos);
    str{2} = ['Min Angle: ', num2str(min_a)];
end

if showLength || showAngle
    if first_plot
        h.angletext = annotation(h.fig, 'textbox', [0.3, 0.72, 0.1, 0.1], ...
            'String', str, 'LineStyle', 'none');
    else
        h.angletext.String = str;
    end
end


hold(h.ax, 'off')
drawnow

view(3)
% pause(0.03)
end


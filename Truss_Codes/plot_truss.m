function [ h ] = plot_truss( truss, opts )
%plot_truss - This plots a truss.   
%   If the input truss has never been plotted before, this function will
%   create a new figure for it. In order to inform the program that a
%   truss has been plotted, it needs to have a valid handle field, truss.h
%   This allows a user to call plotting functions in a loop, such as:
%   |    while(cond)
%   |        truss.h = plot_truss(truss)
%   |    end
%   and the user does not need to worry about special cases for the first
%   time it is plotted, nor about switching figures when multiple
%   independent trusses are plotted
%   Additional plotting options can either be passed in through the opts
%   parameter, or attached as a field in the truss struct. The parameter
%   will override the field if both are defined.
%   plot paramter axisgrid: 
%       0: no axis
%       1: axis but no grid
%       2: axis and grid

if nargin < 2
    if isfield(truss, 'opts')
        opts = truss.opts;
    else
        opts = [];
    end
end
if ~isfield(opts, 'length') || isempty(opts.length)
    opts.length = false;
end
if ~isfield(opts, 'angle') || isempty(opts.angle)
    opts.angle = false;
end
if ~isfield(opts, 'condition') || isempty(opts.condition)
    opts.condition = false;
end
if ~isfield(opts, 'axisgrid') || isempty(opts.length)
    opts.axisgrid = 1;
end
if ~isfield(opts, 'alphalabels') || isempty(opts.alphalabels)
    opts.alphalabels = false;
end


first_plot = true;
if isfield(truss, 'h') && ~isempty(truss.h) && isvalid(truss.h.fig)
    first_plot = false;
    h = truss.h;
else
    % make figure window
    h.fig = figure;
end

figure(h.fig);
cla(h.fig)

elist = truss.elist;
pos = truss.pos;

% plot members
m_p1 = pos(:,elist(:,1));
m_p2 = pos(:,elist(:,2));

m_x1 = m_p1(1,:);
m_x2 = m_p2(1,:);
m_xc = (m_x1 + m_x2)/2;

m_y1 = m_p1(2,:);
m_y2 = m_p2(2,:);
m_yc = (m_y1 + m_y2)/2;

m_z1 = m_p1(3,:);
m_z2 = m_p2(3,:);
m_zc = (m_z1 + m_z2)/2;

if opts.alphalabels
    names = assign_alpha_names(length(elist(:,1)));
else
    names = num2str((1:length(elist(:,1)))');
end

hold on
if first_plot
    line([m_x1;m_x2], [m_y1;m_y2], [m_z1;m_z2],'Color', 'k', 'LineWidth', 1);
    text(m_xc, m_yc, m_zc, names);
else
    line([m_x1;m_x2], [m_y1;m_y2], [m_z1;m_z2],'Color', 'k', 'LineWidth', 1);
    text(m_xc, m_yc, m_zc, names);
end
    
% plot nodes
n_x = pos(1, :);
n_y = pos(2, :);
n_z = pos(3, :);
n_names = [char(32*ones(size(pos, 2),2)), num2str((1:size(pos, 2))')]; % array of 32 is for space padding

if first_plot
    scatter3(n_x, n_y, n_z, 100, 'b','filled');
    text(n_x, n_y, n_z, n_names, 'Color', 'b', 'FontSize', 20);
    axis equal
    if (opts.axisgrid == 2)
        grid on
    elseif (~opts.axisgrid)
        axis off
    end
else 
    scatter3(n_x, n_y, n_z, 100, 'b','filled');
    text(n_x, n_y, n_z, n_names, 'Color', 'b', 'FontSize', 20);
end

if opts.length
    l = lengths(elist, pos);
    str{1} = ['Min Length: ', num2str(min(l)),'   Max Length: ', num2str(max(l))];
end

if opts.angle
    adjm = edgelist_to_adjmatrix(elist);
    [min_a, max_a] = min_member_angle(adjm, pos);
    str{2} = ['Min Angle: ', num2str(min_a),'   Max Angle: ', num2str(max_a)];
end

if opts.condition
    J = length_jacobian(elist, pos);
    S = svd(J);
    str{3} = ['Min Eigenvalue: ', num2str(min(S)),'   Cond: ', num2str(min(S)/max(S))];
end

if opts.angle || opts.condition
    if first_plot
        h.angletext = annotation('textbox', [0.1, 0.9, 0.1, 0.1], 'String', str, 'LineStyle', 'none');
    else
        h.angletext.String = str;
    end
end

hold off
drawnow
end

function names = assign_alpha_names(len)
if len <= 26
    names = char((1:len) + 64)';
elseif len <= 52
    names = char([(1:26) + 64, (27:len) + 70])';
else
    assert(false, 'Too many members');
end
end


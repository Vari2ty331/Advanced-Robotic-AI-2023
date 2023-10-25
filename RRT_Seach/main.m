close all
clear
load('truss_ex.mat')
truss = build_truss(example_truss(1));
% truss = build_truss(octahedron);

xx1 = reshape(truss_ex{1}.pos, [3*length(truss.n),1]);
xx2 = reshape(truss_ex{2}.pos, [3*length(truss.n),1]);
xx3 = reshape(truss_ex{3}.pos, [3*length(truss.n),1]);

% % set constraints F = C*x, x is column vector of all node positions
% C = zeros(6, 3*length(truss.n));
% C(1:3, 1:3) = eye(3); %fix node 1
% C(4, 4) = 1; %fix node 2
% C(5, 6) = 1;
% C(6, 9) = 1; %fix node 3
% x = reshape([truss.n.pos], [3*length(truss.n), 1]);
% F = C*x;

% global dim fixed_index pos_of_fixed_index
global fixed_index pos_of_fixed_index
dim = 3*length(truss.n);
% fixed_index = [1,2,3,4,6,9];
fixed_index = [1:9, 22:24];
pos_of_fixed_index = reshape([truss.n.pos], [dim, 1]);
pos_of_fixed_index = pos_of_fixed_index(fixed_index');

load('initial_config.mat')
initial_config(28) = initial_config(28) + 0.15;
initial_config(29) = initial_config(29) - 0;
initial_config(28) = initial_config(28) - 0.05;
load('final_config.mat')
% initial_config = xx1;
% final_config = xx3;
% initial_config = reshape([truss.n.pos], [dim, 1]);
% final_config = [1,0,0, 0, 0.4853, 0, 0.0753, 0.8196, 0,...
%     0.7890, 0.8608, 0.1728, 0.5972, 0.6336, 0.1493, 0.7286, 0.6656, 0.8542]';
% initial_config = [1
% 0
% 0
% 0
% 0.469793841897677
% 0
% 0.494670161513858
% 0.573741974038710
% 0
% 0.848147374499214
% 0.731458789923221
% 0.216352894269367
% 0.503533134432245
% 0.825463577934968
% 0.0143814737938975
% 0.611765553521229
% 0.332786205098065
% 0.252941656436208];
% final_config = [1
% 0
% 0
% 0
% 0.145072645314145
% 0
% 0.850550434044220
% 0.575319976785517
% 0
% 0.672694318512027
% 0.126103692873379
% 0.944905477340422
% 0.899496693158780
% 0.943115436255366
% 0.103611888530859
% 0.261442822199327
% 0.415314187438446
% 0.573592756345641];

RRT_structure.Start_Node = whole_pos_2_planning_pos(initial_config, fixed_index);
RRT_structure.Goal_Node = whole_pos_2_planning_pos(final_config, fixed_index);
RRT_structure.Dim = dim - length(fixed_index);
RRT_structure.stepsize = 0.01;
RRT_structure.Config_Limit = [-1E5*ones(RRT_structure.Dim, 1), 1E5*ones(RRT_structure.Dim, 1)];
RRT_structure.mixingFactor = 0.1; % 0 for all random


path = BiRRT(RRT_structure, truss);

tmp_truss.m = truss.m;
tmp_truss.n = truss.n;
tmp_truss.h = plot_3Dtruss(tmp_truss);

numpath = size(path.final_path, 2);
numpath2 = 50;
plot_path3 = zeros(size(path.final_path,1), numpath2);
px1 = linspace(0, 1, numpath);
px2 = linspace(0,1,numpath2);
for i=1:size(path.final_path, 1)
    pp = polyfit(px1, path.final_path(i,:), 2);
    plot_path3(i,:) = polyval(pp, px2);
end

for in_ch = 1:5
switch(in_ch)
    case 1 % for split
        p_final = initial_config;
        p_init = p_final;
        p_init(28:30) = p_init(7:9);
        num = 15;
        plot_path = zeros(length(p_init), num);
        for i=1:length(p_init)
            plot_path(i,:) = linspace(p_init(i), p_final(i), num);
        end
        str = 'split';
    case 2
        p_init = initial_config;
        p_final = plot_path3(:,1);
        num = 5;
        plot_path = zeros(length(p_init), num);
        for i=1:length(p_init)
            plot_path(i,:) = linspace(p_init(i), p_final(i), num);
        end
        str = 'reconfiguration (RRT)';
    case 3 % for RRT
%         plot_path = path.final_path;
%         plot_path = path.smooth_path;
        plot_path = plot_path3;
        str = 'reconfiguration (RRT)';
        
%         ss = spline(px1, plot_path, px2);
%         plot_path = ss;
    case 4
        p_init = plot_path3(:,end);
        p_final = final_config;
        num = 5;
        plot_path = zeros(length(p_init), num);
        for i=1:length(p_init)
            plot_path(i,:) = linspace(p_init(i), p_final(i), num);
        end
        str = 'reconfiguration (RRT)';
    case 5 % for merge
        p_init = final_config;
        p_final = p_init;
        p_final(28:30) = p_final(19:21);
        num = 15;
        plot_path = zeros(length(p_init), num);
        for i=1:length(p_init)
            plot_path(i,:) = linspace(p_init(i), p_final(i), num);
        end        
        str = 'merge';
       
end









view([148, 20])
% [az, el] = view;
axis on
% axis(0.8*[-2, 2, -2, 2, -2, 2])
axis(1.25*[-0.1, 1.1, -0.1, 1.1, -0.1, 1.1])
axis manual
grid on
light
note = annotation('textbox', [0.2, 0.85, 0.1, 0.1], 'String', str, 'LineStyle', 'none', 'Tag', 'aaa');
note.FontSize = 50;

xxx = [];
for curr = 1:size(plot_path,2)-1
    x = plot_path(:,curr);
    x_next = plot_path(:,curr+1);
    for ii = length(tmp_truss.n):-1:1
        f{ii} = @(t) (1-t)*x(3*(ii-1)+(1:3)) + t*x_next(3*(ii-1)+(1:3));
    end
    tmp_truss = move_truss(tmp_truss, f, 0:0.1:1, @plot_3Dtruss);
    xxx = [xxx, norm(plot_path(:,curr) - plot_path(:,curr+1))];
end
[i1, i2] = max(xxx)

delete(findall(gcf,'Tag','aaa'))
end
% animate_path

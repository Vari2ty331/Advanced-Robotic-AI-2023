tree = StartTree;

No_FinalNode_Start = length(Final_Path_T);
M_index = 1;

fig = figure('Name','Path Planning Video');
fig.Position = [100 100 700 700];
set(gcf,'color','w');
ax = gca;
ax.FontSize = 15;
ax.FontName = 'Times';
hold on

% camlight

Ground_Number = size(Ground);


% wall_starts = reshape([walls.wall.Start],[3,length([walls.wall.Start])/3])';
% wall_starts = wall_starts(:,1:2)';
% wall_ends = reshape([walls.wall.End],[3,length([walls.wall.End])/3])';
% wall_ends = wall_ends(:,1:2)';
% wall_x = [];
% wall_y = [];
% for i = 1:length(wall_starts)
%     wall_x = [wall_x wall_starts(1,i) wall_ends(1,i) NaN];
%     wall_y = [wall_y wall_starts(2,i) wall_ends(2,i) NaN];
% end
% 
% plot(wall_x,wall_y,'LineWidth',3,'Color','k')
% 
% for index = 1 : Ground_Number(2)
%     [~,GroundObj] = show(Ground(index));
%     GroundObj.FaceColor = [0.7 0.7 0.7];
%     GroundObj.EdgeColor = 'none';
% end

g_x_max = max(Ground(1).Vertices(:,1));
g_x_min = min(Ground(1).Vertices(:,1));
g_y_max = max(Ground(1).Vertices(:,2));
g_y_min = min(Ground(1).Vertices(:,2));

g_x_max = g_x_max+2;
g_x_min = g_x_min-2;
g_y_max = g_y_max+2;
g_y_min = g_y_min-2;

axis equal
axis([g_x_min g_x_max g_y_min g_y_max 0 5]);

plot3(param.goal(1),param.goal(2),param.goal(3),'r.','Markersize',30)
plot3(param.init_com(1),param.init_com(2),param.init_com(3),'g.','Markersize',30)
% view(-63,30)
view(0,90)
grid on

% for i = 2:tree.No_Node-1
%     if tree.T(i).face == 3
%         plot3(tree.T(i).n([1 2 3 1],1),tree.T(i).n([1 2 3 1],2),tree.T(i).n([1 2 3 1],3),'b')
%         plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'b.','Markersize',15 )
%     elseif tree.T(i).face == 4
%         plot3(tree.T(i).n([1 2 3 4 1],1),tree.T(i).n([1 2 3 4 1],2),tree.T(i).n([1 2 3 4 1],3),'b')
%         plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'b.','Markersize',15 )
%     else
%         list = 1:tree.T(i).face;
%         plotList = [list 1];
%         plot3(tree.T(i).n(plotList,1),tree.T(i).n(plotList,2),tree.T(i).n(plotList,3),'b')
%         plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'b.','Markersize',15 )
%     end
% 
%     %     axis equal
%     %     axis([-2 6 -2 6 0 5]);
%     %     ax.FontSize = 15;
%     %     ax.FontName = 'Times';
%     %     ax.XTick = [-2 -1 0 1 2 3 4 5 6];
%     %     ax.YTick = [-2 -1 0 1 2 3 4 5 6];
%     %         MM2(M_index) = getframe(gcf);
%     %         M_index = M_index + 1;
%     % pause(0.01)
% end

for i = 1:No_FinalNode_Start

    if length(Final_Path_T(i).n) == 3
        plot3(Final_Path_T(i).n([1 2 3 1],1),Final_Path_T(i).n([1 2 3 1],2),Final_Path_T(i).n([1 2 3 1],3)+0.02,'g','linewidth',2)
    elseif length(Final_Path_T(i).n) == 4
        plot3(Final_Path_T(i).n([1 2 3 4 1],1),Final_Path_T(i).n([1 2 3 4 1],2),Final_Path_T(i).n([1 2 3 4 1],3)+0.02,'g','linewidth',2)
    else
        list = 1:length(Final_Path_T(i).n);
        plotList = [list 1];
        plot3(Final_Path_T(i).n(plotList,1),Final_Path_T(i).n(plotList,2),Final_Path_T(i).n(plotList,3)+0.02,'g','linewidth',2)
    end
    %    text(Final_Path_T(i).n(1,1), Final_Path_T(i).n(1,2), Final_Path_T(i).n(1,3), num2str(Final_Path_T(i).node(1)),'Color','red','FontSize',14)
    %    text(Final_Path_T(i).n(2,1), Final_Path_T(i).n(2,2), Final_Path_T(i).n(2,3), num2str(Final_Path_T(i).node(2)),'Color','red','FontSize',14)
    %    text(Final_Path_T(i).n(3,1), Final_Path_T(i).n(3,2), Final_Path_T(i).n(3,3), num2str(Final_Path_T(i).node(3)),'Color','red','FontSize',14)

    %     axis equal
    %     axis([-2 6 -2 6 0 5]);
    %     ax.FontSize = 15;
    %     ax.FontName = 'Times';
    %     ax.XTick = [-2 -1 0 1 2 3 4 5 6];
    %     ax.YTick = [-2 -1 0 1 2 3 4 5 6];
%     MM2(M_index) = getframe(gcf);
%     M_index = M_index + 1;
end

finalPath = vertcat(Final_Path_T.path);

plot3(finalPath(:,1),finalPath(:,2),finalPath(:,3),'r-');

axis equal
% axis([-2 6 -2 6 0 5]);
ax.FontSize = 15;
ax.FontName = 'Times';
% ax.XTick = [-2 -1 0 1 2 3 4 5 6];
% ax.YTick = [-2 -1 0 1 2 3 4 5 6];

% plot3(tree.T(end).n([1 2 3 1],1),tree.T(end).n([1 2 3 1],2),tree.T(end).n([1 2 3 1],3),'g')
plot3(tree.T(1).n([1 2 3 1],1),tree.T(1).n([1 2 3 1],2),tree.T(1).n([1 2 3 1],3),'r')
hold off

norm(tree.T(1).foot(3,:) - tree.T(1).n(1,:));

% DateSTR = datestr(now,'yyyymmddHHMMSS');
% fileSTR = append('./recorded_trajectories/',DateSTR);
% mkdir(fileSTR);
% save(append(fileSTR,"/",DateSTR,".mat"));
% video = VideoWriter(append(fileSTR,'/',DateSTR,'_PathPlanning_PRT_Random_ShortTravel.avi'));
% open(video)
% writeVideo(video,MM2);
% close(video)
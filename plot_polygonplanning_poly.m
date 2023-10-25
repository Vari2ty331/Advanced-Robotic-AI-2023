tree = StartTree;

No_FinalNode_Start = length(Final_Path_T);
M_index = 1;
frames = 0;

clear F

fig = figure('Name','Path Planning Video');
fig.Position = [100 100 965 700];
title("Path finding for icosidodecahedron")
% set(gcf,'color','none');
% set(gca,'color','none');
ax = gca;
ax.FontSize = 20;
ax.FontName = 'Times';
axis([-13 12 -13 12 0 5]);
ylabel("y position(m)","FontName","Times","FontSize",20);
xlabel("x position(m)","FontName","Times","FontSize",20);
hold on

% camlight

Ground_Number = size(Ground);


wall_starts = reshape([walls.wall.Start],[3,length([walls.wall.Start])/3])';
wall_starts = wall_starts(:,1:2)';
wall_ends = reshape([walls.wall.End],[3,length([walls.wall.End])/3])';
wall_ends = wall_ends(:,1:2)';
wall_x = [];
wall_y = [];
for i = 1:length(wall_starts)
    wall_x = [wall_x wall_starts(1,i) wall_ends(1,i) NaN];
    wall_y = [wall_y wall_starts(2,i) wall_ends(2,i) NaN];
end

plot(wall_x,wall_y,'LineWidth',5,'Color','k')

for index = 1 : Ground_Number(2)
    [~,GroundObj] = show(Ground(index));
    GroundObj.FaceColor = [0.8 0.8 0.8];
    GroundObj.EdgeColor = 'none';
end

g_x_max = max(Ground(1).Vertices(:,1));
g_x_min = min(Ground(1).Vertices(:,1));
g_y_max = max(Ground(1).Vertices(:,2));
g_y_min = min(Ground(1).Vertices(:,2));

g_x_max = g_x_max+1;
g_x_min = g_x_min-1;
g_y_max = g_y_max+1;
g_y_min = g_y_min-1;

axis equal
axis([g_x_min g_x_max g_y_min g_y_max 0 5]);


if exist('obstacle','var') ~= 0
    Obstacle_1 = [  obstacle{1}(:,1), obstacle{1}(:,2), [0; 0; 0; 0; 0;];
        obstacle{1}(:,1), obstacle{1}(:,2), [4; 4; 4; 4; 4;];  ];
    Obstacle_1_Mesh = collisionMesh(Obstacle_1);
    [~,ObstacleObj1] = show(Obstacle_1_Mesh);
    ObstacleObj1.FaceColor = [1 0.7 0];
    ObstacleObj1.EdgeColor = 'none';


    Obstacle_2 = [  obstacle{2}(:,1), obstacle{2}(:,2), [0; 0; 0; 0; 0;];
        obstacle{2}(:,1), obstacle{2}(:,2), [4; 4; 4; 4; 4;];  ];
    Obstacle_2_Mesh = collisionMesh(Obstacle_2);
    [~,ObstacleObj2] = show(Obstacle_2_Mesh);
    ObstacleObj2.FaceColor = [1 0.7 0];
    ObstacleObj2.EdgeColor = 'none';
end

plot3(param.goal(1),param.goal(2),param.goal(3),'r.','Markersize',30)
plot3(param.init_com(1),param.init_com(2),param.init_com(3),'g.','Markersize',30)

dummy1 = plot3([0.01 0.02],[0.01 0.02],[-1 -1],'color','#D0006B','LineWidth',2);
dummy2 = plot3([0.01 0.02],[0.01 0.02],[-1 -1],'color','#FFAD5C','LineWidth',2);
legend([dummy1 dummy2],{'Ear clipping','Rapid Triangulation'},'location','bestoutside','FontSize',15,'AutoUpdate','off')
% delete([dummy1 dummy2]);
% view(-63,30)
view(0,90)
grid on

for i = 2:tree.No_Node-1
    if tree.T(i).face == 3
        p1 = plot3(tree.T(i).n([1 2 3 1],1),tree.T(i).n([1 2 3 1],2),tree.T(i).n([1 2 3 1],3),'Color','#008ecc','LineWidth',0.1);
        p2 = plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'Color','#008ecc','Markersize',3,'LineStyle','none' );
    elseif tree.T(i).face == 4
        p1 = plot3(tree.T(i).n([1 2 3 4 1],1),tree.T(i).n([1 2 3 4 1],2),tree.T(i).n([1 2 3 4 1],3),'Color','#008ecc','LineWidth',0.1);
        p2 = plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'Color','#008ecc','Markersize',3,'LineStyle','none' );
    else
        list = 1:tree.T(i).face;
        plotList = [list 1];    
        p1 = plot3(tree.T(i).n(plotList,1),tree.T(i).n(plotList,2),tree.T(i).n(plotList,3),'Color','#008ecc','LineWidth',0.1);
        p2 = plot3(tree.T(i).foot(:,1) , tree.T(i).foot(:,2) , tree.T(i).foot(:,3),'Color','#008ecc','Markersize',3,'LineStyle','none');
    end
    % p1.Color(4) = 0.6;
    % p2.Color(4) = 0.6;

    %     axis equal
    %     axis([-2 6 -2 6 0 5]);
    %     ax.FontSize = 15;
    %     ax.FontName = 'Times';
    %     ax.XTick = [-2 -1 0 1 2 3 4 5 6];
    %     ax.YTick = [-2 -1 0 1 2 3 4 5 6];
    %         MM2(M_index) = getframe(gcf);
    %         M_index = M_index + 1;
    % pause(0.000001)
    % frames = frames + 1;
    % F(frames) = getframe(gcf);
end

for i = 1:No_FinalNode_Start

    if length(Final_Path_T(i).n) == 3
        plot3(Final_Path_T(i).n([1 2 3 1],1),Final_Path_T(i).n([1 2 3 1],2),Final_Path_T(i).n([1 2 3 1],3)+0.02,'g','linewidth',1.2)
    elseif length(Final_Path_T(i).n) == 4
        plot3(Final_Path_T(i).n([1 2 3 4 1],1),Final_Path_T(i).n([1 2 3 4 1],2),Final_Path_T(i).n([1 2 3 4 1],3)+0.02,'g','linewidth',1.2)
    else
        list = 1:length(Final_Path_T(i).n);
        plotList = [list 1];
        plot3(Final_Path_T(i).n(plotList,1),Final_Path_T(i).n(plotList,2),Final_Path_T(i).n(plotList,3)+0.02,'g','linewidth',1.2)
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
% frames = frames + 1;
% F(frames) = getframe(gcf);
end

finalPath = vertcat(Final_Path_T.path);
finalPath2 = vertcat(Final_Path_T.path2);
finalPath3 = vertcat(Final_Path_T.path3);
for pathitr = 1:length(finalPath)-1
    p1(pathitr) = plot3(finalPath([pathitr pathitr+1],1),finalPath([pathitr pathitr+1],2),finalPath([pathitr pathitr+1],3),'color','#D0006B','LineWidth',1.5);
    % frames = frames + 1;
    % F(frames) = getframe(gcf);
end



for pathitr = 1:length(finalPath)-1
    p2(pathitr) = plot3(finalPath3([pathitr pathitr+1],1),finalPath3([pathitr pathitr+1],2),finalPath3([pathitr pathitr+1],3),'color','#FFAD5C','LineWidth',2.3);
    % frames = frames + 1;
    % F(frames) = getframe(gcf);
end
% p1 = plot3(finalPath(:,1),finalPath(:,2),finalPath(:,3),'color','#D0006B','LineWidth',1.5);
% p2 = plot3(finalPath3(:,1),finalPath3(:,2),finalPath3(:,3),'color','#FFAD5C','LineWidth',1.5);
% plot3(finalPath2(:,1),finalPath2(:,2),finalPath2(:,3),'c-');

% show(p1);

aaa = 10.5-6;
bbb = 9.8-6;
ccc = 10.2-6;

format shortE
text1 = text(13.5,aaa-2,0,"Computation Time");
text11 = text(14.2,bbb-2,0,"(Brute Force)");
text111 = text(19.5,ccc-2,0," : " + num2str(pathInfo.brute.time,'%.3e')+ " s");
text2 = text(13.5,aaa-2-2,0,"Computation Time");
text22 = text(15,bbb-2-2,0,"(Rapid)");
text222 = text(19.5,ccc-2-2,0," : " + num2str(pathInfo.rapid.time,'%.3e')+ " s");
text3 = text(13.5,aaa-2-2-2,0,"Shortest Distance");
text33 = text(14.2,bbb-2-2-2,0,"(Brute Force)");
text333 = text(19.5,ccc-2-2-2,0," : "+num2str(pathInfo.brute.shortestLength)+ " m");
text5 = text(13.5,aaa-2-2-2-2,0,"Centered Distance");
text55 = text(15.2,bbb-2-2-2-2,0,"(Rapid)");
text555 = text(19.5,ccc-2-2-2-2,0," : "+num2str(pathInfo.rapid.Length)+ " m");

text1.FontSize = 12;
text11.FontSize = 12;
text111.FontSize = 12;
text2.FontSize = 12;
text22.FontSize = 12;
text222.FontSize = 12;
text3.FontSize = 12;
text33.FontSize = 12;
text333.FontSize = 12;
text5.FontSize = 12;
text55.FontSize = 12;
text555.FontSize = 12;

for asdf = 1:120
% F(frames+asdf) = getframe(gcf);
end
% % text4 = text(2,-13.5,0,"Centered Distance(Brute Force): "+num2str(pathInfo.brute.closestLength)+ " m");
% text5 = text(13,7.5,0,"Centered Distance(Rapid): "+num2str(pathInfo.rapid.Length)+ " m");

% axis equal
% axis([-2 6 -2 6 0 5]);
% ax.FontSize = 20;
% ax.FontName = 'Times';
% ax.XTick = [-2 -1 0 1 2 3 4 5 6];
% ax.YTick = [-2 -1 0 1 2 3 4 5 6];

% plot3(tree.T(end).n([1 2 3 1],1),tree.T(end).n([1 2 3 1],2),tree.T(end).n([1 2 3 1],3),'g')
plot3(tree.T(1).n([1 2 3 1],1),tree.T(1).n([1 2 3 1],2),tree.T(1).n([1 2 3 1],3),'r')
hold off

norm(tree.T(1).foot(3,:) - tree.T(1).n(1,:));
% axis off





% gca
% DateSTR = datestr(now,'yyyymmddHHMMSS');
% fileSTR = append('./recorded_trajectories/',DateSTR);
% mkdir(fileSTR);
% save(append(fileSTR,"/",DateSTR,".mat"));
% video = VideoWriter(append(fileSTR,'/',DateSTR,'_PathPlanning_PRT_Random_ShortTravel.avi'));
% open(video)
% writeVideo(video,MM2);
% close(video)


%%
% 
% writerObj = VideoWriter('icododacahedron.avi');
% writerObj.FrameRate = 150;
% writerObj.Quality = 100;
% open(writerObj);
% for i=1:length(F)
%     frame = F(i) ;
%     writeVideo(writerObj, frame);
% end

% close(writerObj);
clc
clear
close all

points = [-0.577350269189626	0	0;
0.288675134594813	0.500000000000000	0;
0.288675134594813	-0.500000000000000	0;
0.577350269189626	0	0.816496580927726;
-0.288675134594813	-0.500000000000000	0.816496580927726;
-0.288675134594813	0.500000000000000	0.816496580927726;
0.577350269189626	0	0.816496580927726;
0.288675134594813	0.500000000000000	0;
-0.288675134594813	0.500000000000000	0.816496580927726;
-0.577350269189626	0	0;
-0.288675134594813	-0.500000000000000	0.816496580927726;
0.288675134594813	-0.500000000000000	0;
-0.577350269189626	0	0;];
points2 =  points(1:6,:);
fig = figure('Name','asdf');
fig.Position = [100 100 700 700];
set(gcf,'color','w');
ax = gca;
ax.FontSize = 15;
ax.FontName = 'Times';
view(-63,30)
hold on


plot3(points(:,1),points(:,2),points(:,3),'-','LineWidth',1.5,'Color','black')
plot3(points2(:,1),points2(:,2),points2(:,3),'.','MarkerSize',30)
set(gcf, 'color', 'none');    
set(gca, 'color', 'none');
axis equal
grid on
axis([-1 1 -1 1 0 1]);
exportgraphics(gcf,'transparent.eps',...   % since R2020a
    'ContentType','vector',...
    'BackgroundColor','none')
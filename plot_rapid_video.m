fig = figure();
fig.Position = [632 458 440 440];
% t = tiledlayout(x,y);
% t.TileSpacing = 'compact';
% t.Padding = 'compact';
plot(polygon([1 2 3 4 5 6 7 8 1],1),polygon([1 2 3 4 5 6 7 8 1],2),'k-','linewidth',2);
hold on
plot(polygon(startIndex,1),polygon(startIndex,2),'-',LineWidth=5,Color="#6FCE67")
plot(polygon(endIndex,1),polygon(endIndex,2),'-',LineWidth=5,Color="#FFAD5C")
quiver(startCenter(1),startCenter(2),startEndVector(1),startEndVector(2),'AutoScale','off','Color',"#FFC1DE",'LineWidth',2)
axis square
% outSets(132).minPathPerm = [4 5 6 2 1 3];


% nexttile
hold on
% set = outSets((i-1)*y+j);


a = plot(polygon([3 5],1),polygon([3 5],2),':k');
b = plot(polygon([4 2],1),polygon([4 2],2),':k');
text1 = text(0,1.3,"Compare Adjacent nodes");

% pause(0.5)
F(1) = getframe(gcf);

centerdota = plot(sum(polygon([3 4 5],1))/3,sum(polygon([3 4 5],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
centerdotb = plot(sum(polygon([3 4 2],1))/3,sum(polygon([3 4 2],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(2) = getframe(gcf);

perpendiculara = plot([sum(polygon([3 4 5],1))/3,-0.193648453354488],[sum(polygon([3 4 5],2))/3,sum(polygon([3 4 5],2))/3],'-b',LineWidth=0.7);
perpendicularb = plot([sum(polygon([3 4 2],1))/3,-0.193648453354488],[sum(polygon([3 4 2],2))/3,sum(polygon([3 4 2],2))/3],'-b',LineWidth=0.7);
text2 = text(0,1.2,"CCW is closer than CW");
% pause(0.5)
F(3) = getframe(gcf);
delete([perpendiculara,perpendicularb,centerdotb,b,text2,text1]);
% pause(0.5)
F(4) = getframe(gcf);
text1 = text(0,1.3,"Change Base edge and continue");
newBase = plot(polygon([3 5],1),polygon([3 5],2),Color="#6FCE67",LineStyle=":",LineWidth=1.5);
delete(a);
% pause(0.5)
F(5) = getframe(gcf);
a = plot(polygon([6 3],1),polygon([6 3],2),':k');
b = plot(polygon([5 2],1),polygon([5 2],2),':k');
% pause(0.5)
F(6) = getframe(gcf);
centerdota = plot(sum(polygon([3 5 6],1))/3,sum(polygon([3 5 6],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
centerdotb = plot(sum(polygon([3 5 2],1))/3,sum(polygon([3 5 2],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(7) = getframe(gcf);
perpendiculara = plot([sum(polygon([3 5 6],1))/3,-0.193648453354488],[sum(polygon([3 5 6],2))/3,sum(polygon([3 5 6],2))/3],'-b',LineWidth=0.7);
perpendicularb = plot([sum(polygon([3 5 2],1))/3,-0.193648453354488],[sum(polygon([3 5 2],2))/3,sum(polygon([3 5 2],2))/3],'-b',LineWidth=0.7);
% pause(0.5)
F(8) = getframe(gcf);
newBase = plot(polygon([3 6],1),polygon([3 6],2),Color="#6FCE67",LineStyle=":",LineWidth=1.5);
% pause(0.5)
F(9) = getframe(gcf);
delete([perpendiculara,perpendicularb,centerdotb,b,text1]);
% pause(0.5)
F(10) = getframe(gcf);
text1 = text(0,1.3,"Query edge reached goal");
text2 = text(0,1.2,"Create route without comparing");
plot(polygon([6 2],1),polygon([6 2],2),':k');
plot(sum(polygon([3 2 6],1))/3,sum(polygon([3 2 6],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(11) = getframe(gcf);
plot(polygon([6 1],1),polygon([6 1],2),':k');
plot(sum(polygon([2 1 6],1))/3,sum(polygon([2 1 6],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(12) = getframe(gcf);
plot(polygon([6 8],1),polygon([6 8],2),':k');
% pause(0.5)
F(13) = getframe(gcf);
plot(sum(polygon([8 1 6],1))/3,sum(polygon([8 1 6],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(14) = getframe(gcf);
plot(sum(polygon([8 7 6],1))/3,sum(polygon([8 7 6],2))/3,'Marker','o',MarkerFaceColor='k',Color='k',MarkerSize=4);
% pause(0.5)
F(15) = getframe(gcf);

delete([text1,text2]);
text1 = text(0,1.3,"Route Found");

% pause(0.5)

centerArry = vertcat(set.triangles.center);
centerArry = centerArry(set.minPathPerm,:);
% plot(centerArry(:,1),centerArry(:,2),'b-')
temp = centerArry;
temp2 = centerArry;
temp(1,:) = [];
temp2(end,:) = [];
centerVectors = temp - temp2;
quivers = quiver(temp2(:,1),temp2(:,2),centerVectors(:,1),centerVectors(:,2),'LineWidth',1.5,'AutoScale','off','color','#8FAADC');
totallength = sum(diag(centerVectors*centerVectors'));
text2 = text(0,1.2,"Path Distance : " + totallength);
F(16) = getframe(gcf);
% quivers = quiver(temp2(:,1),temp2(:,2),centerVectors(:,1),centerVectors(:,2),'LineWidth',1.5,'AutoScale','off','color','#8FAADC');
% text1 = text(0,1.3,"Triangulantion Set Num : " + num2str((i-1)*y+j));
% text2 = text(0,1.2,"Path distance : " + num2str(set.minPath));
% plot(polygon([1:length(polygon) 1],1),polygon([1:length(polygon) 1],2),'-k','LineWidth',2);
hold off
% F((i-1)*y+j) = getframe(gcf);
% pause(0.05);
% if i ~=x || j ~= y
%     delete(plot_triangulation);
%     delete(quivers);
%     delete(text1);
%     % delete(text2);
% end

writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 0.8;
writerObj.Quality = 100;
open(writerObj);
for i=1:length(F)
    frame = F(i) ;
    writeVideo(writerObj, frame);
end
close(writerObj);
% exportgraphics(gcf,'triangulations.pdf','ContentType','vector')
fig = figure();
fig.Position = [632 458 440 440];
x = 11;
y = 12;
% t = tiledlayout(x,y);
% t.TileSpacing = 'compact';
% t.Padding = 'compact';
plot(polygon([1 2 3 4 5 6 7 8 1],1),polygon([1 2 3 4 5 6 7 8 1],2),'k-','linewidth',2);
hold on
plot(polygon(startIndex,1),polygon(startIndex,2),'-',LineWidth=5,Color="#6FCE67")
plot(polygon(endIndex,1),polygon(endIndex,2),'-',LineWidth=5,Color="#FFAD5C")
axis square
outSets(132).minPathPerm = [4 5 6 2 1 3];
for i = 1:x
    for j = 1:y
        % nexttile
        hold on
        set = outSets((i-1)*y+j);
        for k = 1:length(set.triangles)
            coords = set.triangles(k).coords;
            plot_triangulation(k) = plot(coords([1 2 3 1],1),coords([1 2 3 1],2),'k:');

        end
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
        % text(0,1.2,"path length : " + num2str(totallength),"FontName","Times","FontSize",16);
        text1 = text(0.2,1.3,"Triangulantion Set Num : " + num2str((i-1)*y+j));
        text2 = text(0.2,1.2,"Path distance : " + totallength);
        % plot(polygon([1:length(polygon) 1],1),polygon([1:length(polygon) 1],2),'-k','LineWidth',2);
        hold off
        F((i-1)*y+j) = getframe(gcf);
        % pause(0.05);
        if i ~=x || j ~= y
            delete(plot_triangulation);
            delete(quivers);
            delete(text1);
            delete(text2);
        end
    end
end

writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 22;
writerObj.Quality = 100;
open(writerObj);
for i=1:length(F)
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
close(writerObj);
% exportgraphics(gcf,'triangulations.pdf','ContentType','vector')
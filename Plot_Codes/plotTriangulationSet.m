function plotTriangulationSet(set,polygon,startEnd,setVector,time)

if nargin<2

    figure()
    hold on

    for i = 1:length(set.triangles)
        coords = set.triangles(i).coords;
        plot(coords([1 2 3 1],1),coords([1 2 3 1],2),'-');

    end

elseif nargin<4
    fig = figure();
    fig.Position = [632 458 440 440];
    hold on

    for i = 1:length(set.triangles)
        coords = set.triangles(i).coords;
        plot(coords([1 2 3 1],1),coords([1 2 3 1],2),':k');
        centers = set.triangles(i).center;
        % text(centers(1),centers(2),num2str(i));
    end

    % for i = 1:length(polygon)
    %     text(polygon(i,1),polygon(i,2)+0.1,num2str(i));
    % end

    centerArry = vertcat(set.triangles.center);
    centerArry = centerArry(set.minPathPerm,:);
    % plot(centerArry(:,1),centerArry(:,2),'b-')
    temp = centerArry;
    temp2 = centerArry;
    temp(1,:) = [];
    temp2(end,:) = [];
    centerVectors = temp - temp2;
    quiver(temp2(:,1),temp2(:,2),centerVectors(:,1),centerVectors(:,2),'LineWidth',1.5,'AutoScale','off','color','#8FAADC');
    totallength = sum(diag(centerVectors*centerVectors'));
    % text(0,1.2,"path length : " + num2str(totallength),"FontName","Times","FontSize",16);
    plot(polygon([1:length(polygon) 1],1),polygon([1:length(polygon) 1],2),'-k','LineWidth',2);
    
    if nargin == 3
        startIdx = startEnd(1,:);
        endIdx = startEnd(2,:);
        plot(polygon(startIdx,1),polygon(startIdx,2),'-',LineWidth=5,Color="#6FCE67")
        plot(polygon(endIdx,1),polygon(endIdx,2),'-',LineWidth=5,Color="#FFAD5C")
    end
    text1 = text(0.2,1.3,"Triangulantion Set Num : 17");
    text2 = text(0.2,1.2,"Path distance : " + totallength);
    % exportgraphics(gcf,'triangulations_min.emf','ContentType','vector')
else
    figure()
    hold on
    title('Your text here')

    if setVector == 1
        startIdx = startEnd(1,:);
        endIdx = startEnd(2,:);
        startCenter = sum(polygon(startIdx,:))/2;
        endCenter = sum(polygon(endIdx,:))/2;
        startEndVector = endCenter - startCenter;
        quiver(startCenter(:,1),startCenter(:,2),startEndVector(:,1),startEndVector(:,2),'off','color','#8FAADC','LineWidth',2);
    end

    for i = 1:length(set.triangles)
        coords = set.triangles(i).coords;
        plot(coords([1 2 3 1],1),coords([1 2 3 1],2),'--k');
        centers = set.triangles(i).center;
        % text(centers(1),centers(2),num2str(i));
    end

    % for i = 1:length(polygon)
    %     text(polygon(i,1),polygon(i,2)+0.1,num2str(i));
    % end

    centerArry = vertcat(set.triangles.center);
    centerArry = centerArry(set.minPathPerm,:);
    % plot(centerArry(:,1),centerArry(:,2),'b-')
    temp = centerArry;
    temp2 = centerArry;
    temp(1,:) = [];
    temp2(end,:) = [];
    centerVectors = temp - temp2;
    quiver(temp2(:,1),temp2(:,2),centerVectors(:,1),centerVectors(:,2),'LineWidth',1.5,'AutoScale','off','color','#8FAADC');
    plot(polygon([1:length(polygon) 1],1),polygon([1:length(polygon) 1],2),'-k','LineWidth',2);
    if nargin >= 3
        startIdx = startEnd(1,:);
        endIdx = startEnd(2,:);
        plot(polygon(startIdx,1),polygon(startIdx,2),'-g',LineWidth=5)
        plot(polygon(endIdx,1),polygon(endIdx,2),'-r',LineWidth=5)
    end

    text(0,0,"computation time : " + num2str(time) + " secs","FontName","Times","FontSize",16);
    totallength = sum(diag(centerVectors*centerVectors'));
    text(0,-0.5,"path length : " + num2str(totallength),"FontName","Times","FontSize",16);


end
axis square
ax = gca;
% ax.FontSize = 15;
% ax.FontName = 'Times';
% set(gcf,'color','none');
% set(gca,'color','none');
end
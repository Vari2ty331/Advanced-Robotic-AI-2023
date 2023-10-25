clc, clear
close all
wholeStartTime = tic;

%%
for counter = 1:1
    randIndex = [1:8;2:8,1];
    randIndex = randIndex(:,randperm(8,2));
    sets.triangles.coords = [];

    % polygon = createRandomPolygon(8);
    polygon = load('polygon.mat');
    polygon = polygon.polygon;
    catalan = findCatalanTriangles(polygon);

    %%
    index = [1 2 3 4 5 6 7 8];

    outSets = outputSetOctagon(polygon,index,sets);

    %%
    % startIndex = randIndex(:,1)';
    % endIndex = randIndex(:,2)';
    startIndex = [3,4];
    endIndex = [6,7];

    startToEndVector = [sum(polygon(endIndex,:))/2 - sum(polygon(startIndex,:))/2,0];



    for setItr = 1:length(outSets)
        startTriangle = [];
        endTriangle = [];
        centerVectors = vertcat(outSets(setItr).triangles.center);
        centerVectors = [centerVectors,zeros(length(centerVectors),1)];
        centerVectors = centerVectors - [sum(polygon(endIndex,:))/2 , 0];


        for i = 1:length(outSets(setItr).triangles)
            if ismember(polygon(startIndex,:),outSets(setItr).triangles(i).coords)
                startTriangle = i;
            elseif ismember(polygon(endIndex,:),outSets(setItr).triangles(i).coords)
                endTriangle = i;
            end
            centerDistances(i) = norm(cross(centerVectors(i,:),startToEndVector))/norm(startToEndVector);
        end

        outSets(setItr).meanCenterDistanceToVector = mean(centerDistances);

        if isempty(startTriangle) || isempty(endTriangle)
            continue;
        end

        pathIdx = 1:length(polygon)-2;
        pathIdx([startTriangle,endTriangle]) = [];
        pathPerm = flipud(perms(pathIdx));
        startTriangle = ones(length(pathPerm),1)*startTriangle;
        endTriangle = ones(length(pathPerm),1)*endTriangle;
        pathPerm = [startTriangle,pathPerm,endTriangle];
        centers = [];
        pathPermSize = size((pathPerm));
        for i = 1:pathPermSize(1)
            centers = vertcat(outSets(setItr).triangles(pathPerm(i,:)).center);
            pathDist(i) = getDistance(centers);
        end

        [minPath,minPathIdx] = min(pathDist);
        outSets(setItr).minPath = minPath;
        outSets(setItr).minPathPerm = pathPerm(minPathIdx,:);
    end

    minPaths = vertcat(outSets.minPath);
    [minMinPath,minMinPathIdx] = min(minPaths);
    meanDistances = vertcat(outSets.meanCenterDistanceToVector);
    [minMean, minMeanIdx] = min(meanDistances);

end
wholeTime = toc(wholeStartTime)
plotTriangulationSet(outSets(minMinPathIdx),polygon,[startIndex;endIndex])
% plotTriangulationSet(outSets(minMeanIdx),polygon,[startIndex;endIndex])
% minMinPath
% outSets(minMeanIdx).minPath
% outSets(minMeanIdx).minPath-minMinPath
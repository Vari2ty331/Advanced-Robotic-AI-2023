function [outSets,minMinPath,minMinPathIdx,minMean, minMeanIdx,loopTime] = bruteForceFunction(polygon,startIndex,endIndex)


sets.triangles.coords = [];

index = 1:length(polygon);

outSets = outputSetPentagon(polygon,index,sets);

%%
% startIndex = [1 2];
% endIndex = [2 3];


startToEndVector = [sum(polygon(endIndex,:))/2 - sum(polygon(startIndex,:))/2];
time = tic;
for setItr = 1:length(outSets)
    startTriangle = [];
    endTriangle = [];
    centerVectors = vertcat(outSets(setItr).triangles.center);
    % centerVectors = [centerVectors,zeros(length(centerVectors),1)];
    centerVectors = centerVectors - sum(polygon(endIndex,:))/2;
    for i = 1:length(outSets(setItr).triangles)
        if ismember(polygon(startIndex,:),outSets(setItr).triangles(i).coords)
            startTriangle = i;
        elseif ismember(polygon(endIndex,:),outSets(setItr).triangles(i).coords)
            endTriangle = i;
        end
        centerDistances(i) = norm(cross(centerVectors(i,:),startToEndVector))/norm(startToEndVector);
    end
    outSets(setItr).meanCenterDistanceToVector = mean(centerDistances);

    if isempty(startTriangle) || isempty(endTriangle) %|| startTriangle == endTriangle
        continue;
    end

    polygonSize = size(polygon);
    pathIdx = 1:(polygonSize(1)-2);
    pathIdx([startTriangle,endTriangle]) = [];
    pathPerm = flipud(perms(pathIdx));
    startTriangle = ones(length(pathPerm),1)*startTriangle;
    endTriangle = ones(length(pathPerm),1)*endTriangle;
    pathPerm = [startTriangle,pathPerm,endTriangle];
    pathPermSize = size((pathPerm));
    for i = 1:pathPermSize(1)
        centers = vertcat(outSets(setItr).triangles(pathPerm(i,:)).center);
        pathDist(i) = getDistance(centers);
    end

    [minPath,minPathIdx] = min(pathDist);
    outSets(setItr).minPath = minPath;
    outSets(setItr).minPathPerm = pathPerm(minPathIdx,:);
end
loopTime = toc(time);

minPaths = vertcat(outSets.minPath);
[minMinPath,minMinPathIdx] = min(minPaths);
meanDistances = vertcat(outSets.meanCenterDistanceToVector);
[minMean, minMeanIdx] = min(meanDistances);



end
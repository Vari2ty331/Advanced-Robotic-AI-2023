clc, clear
close all
wholeStartTime = tic;

for counter = 1:1000
    clear rapidTime bruteTime
    randIndex = [1:8;2:8,1];
    randIndex = randIndex(:,randperm(8,2));
    polygon = createRandomPolygon(8);
    polygon = [polygon zeros(8,1)];
    index = [1 2 3 4 5 6 7 8];
    startIndex = randIndex(:,1)';
    endIndex = randIndex(:,2)';
    tic;
    [outSets{counter},minMinPath{counter},minMinPathIdx{counter},minMean{counter}, minMeanIdx{counter},loopTime{counter}] = bruteForceFunction(polygon,startIndex,endIndex);
    bruteTime_arry(counter) = toc;
    tic;
    [outSets2{counter},minMinPath2{counter},minMinPathIdx2{counter}] = rapidPolygonPathFunction(polygon,startIndex,endIndex);
    % outSets2{counter}.minPathPerm = 1:length(outSets2.triangles);
    rapidTime_arry(counter) = toc;

end

wholeStartTime = toc(wholeStartTime);


%%
plotTriangulationSet(outSets(minMinPathIdx),polygon,[startIndex;endIndex],1,bruteTime);
plotTriangulationSet(outSets(minMeanIdx),polygon,[startIndex;endIndex],1,bruteTime);
plotTriangulationSet(outSets2,polygon,[startIndex;endIndex],1,rapidTime);
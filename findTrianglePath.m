function [tree_with_path,pathInfo] = findTrianglePath(Final_Path_T)

tree = Final_Path_T;

for i = 2:length(tree)-1
    polygon = tree(i).n;
    if length(polygon) < 4
        tree(i).path = sum(polygon)/3;
        tree(i).path2 = sum(polygon)/3;
        tree(i).path3 = sum(polygon)/3;
    else
        startVertex = findMatchingVertex(Final_Path_T(i-1).n,Final_Path_T(i).n);
        endVertex   = findMatchingVertex(Final_Path_T(i).n,Final_Path_T(i+1).n);
        [temp,startIndex] = ismember(startVertex,Final_Path_T(i).n,'rows');
        [temp,endIndex] = ismember(endVertex,Final_Path_T(i).n,'rows');
        [outSets,minMinPath,minMinPathIdx,minMean, minMeanIdx,loopTimeBruteForce] = bruteForceFunction(polygon,startIndex,endIndex);
        [outSets2,minMinPath2,minMinPathIdx2,loopTimeRapid] = rapidPolygonPathFunction(polygon,startIndex,endIndex);
        tree(i).path = vertcat(outSets(minMinPathIdx).triangles.center);
        tree(i).path2 = vertcat(outSets2.triangles.center);
        tree(i).path3 = vertcat(outSets(minMeanIdx).triangles.center);
    end    
end

tree(1).path = sum(tree(1).n,1)/3;
tree(end).path = sum(tree(end).n,1)/length(tree(end).n);
tree(1).path2 = sum(tree(1).n,1)/3;
tree(end).path2 = sum(tree(end).n,1)/length(tree(end).n);
tree(1).path = sum(tree(1).n,1)/3;
tree(end).path = sum(tree(end).n,1)/length(tree(end).n);
tree(1).path3 = sum(tree(1).n,1)/3;
tree(end).path3 = sum(tree(end).n,1)/length(tree(end).n);

pathInfo.brute.time = loopTimeBruteForce;
pathInfo.rapid.time = loopTimeRapid;
pathInfo.brute.shortestLength = getDistance(vertcat(tree.path));
pathInfo.brute.closestLength = getDistance(vertcat(tree.path2));
pathInfo.rapid.Length = getDistance(vertcat(tree.path3));


tree_with_path = tree;
end
clc, clear

n = 6;
randPoly = createRandomPolygon(n);
pivotPositions = randperm(n,2);


for i = 1:n
    polygons.vertices(i).coords = randPoly(i);
    polygons.vertices(i).index = i;
    if i ~= n
        polygons.edge(i).index = [i i+1];
    else
        polygons.edge(i).index = [i 1];
    end
    if i ~= pivotPositions(1) && i ~= pivotPositions(2)
        polygons.edge(i).isPivot = 0;
    elseif i == pivotPositions(1)
        polygons.edge(i).isPivot = 1;
    else
        polygons.edge(i).isPivot = 2;
    end
end



polygons.edgeStart = pivotPositions(1);
polygons.edgetEnd = pivotPositions(2);

polygons = findCatalanTriangles(polygons);
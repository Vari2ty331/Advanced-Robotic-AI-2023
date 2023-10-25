clear,clc


set1(1).triangles(1).coords = createRandomPolygon(3);
set1(1).triangles(2).coords = createRandomPolygon(3);

set1(2).triangles(1).coords = createRandomPolygon(3);
set1(2).triangles(2).coords = createRandomPolygon(3);
% 
% set1(3).triangles(1).coords = createRandomPolygon(3);
% set1(3).triangles(2).coords = createRandomPolygon(3);
% set1(3).triangles(3).coords = createRandomPolygon(3);


set2(1).triangles(1).coords = createRandomPolygon(3);
set2(1).triangles(2).coords = createRandomPolygon(3);
set2(1).triangles(3).coords = createRandomPolygon(3);

set2(2).triangles(1).coords = createRandomPolygon(3);
set2(2).triangles(2).coords = createRandomPolygon(3);
set2(2).triangles(3).coords = createRandomPolygon(3);

set2(3).triangles(1).coords = createRandomPolygon(3);
set2(3).triangles(2).coords = createRandomPolygon(3);
set2(3).triangles(3).coords = createRandomPolygon(3);

outset.triangles = [];

for i = 1:length(set1)
    for j = 1:length(set2)
        temp((i-1)*3+j).triangles = [set1(i).triangles ,set2(j).triangles];
        % outset((i-1)*3+j) = [set1(i) ,set2(j)];
    end
end


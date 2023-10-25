function setOut = outputSetSquare(polygon,index,sets)

set1 = addTriangleSets(outputSetTriangle(polygon,index([1 2 3]),sets) , outputSetTriangle(polygon,index([1 3 4]),sets));
set2 = addTriangleSets(outputSetTriangle(polygon,index([1 2 4]),sets) , outputSetTriangle(polygon,index([2 3 4]),sets));

sets1(1) = set1;
sets1(2) = set2;

setOut = sets1;
end
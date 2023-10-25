function setOut = outputSetPentagon(polygon,index,sets)



setOut1 = addTriangleSets(outputSetTriangle(polygon,index([1 2 5]),sets),outputSetSquare(polygon,index([2 3 4 5]),sets));
setOut2 = addTriangleSets(outputSetTriangle(polygon,index([1 2 4]),sets),outputSetTriangle(polygon,index([1 4 5]),sets),outputSetTriangle(polygon,index([2 3 4]),sets));
setOut3 = addTriangleSets(outputSetTriangle(polygon,index([1 2 3]),sets),outputSetSquare(polygon,index([1 3 4 5]),sets));


setOut(1).triangles = setOut1(1).triangles;
setOut(2).triangles = setOut1(2).triangles;
setOut(3).triangles = setOut2.triangles;
setOut(4).triangles = setOut3(1).triangles;
setOut(5).triangles = setOut3(2).triangles;

end 
function setOut = outputSetHexagon(polygon,index,sets)

setOut1 = addTriangleSets(outputSetTriangle(polygon,index([1 2 3]),sets),outputSetPentagon(polygon,index([1 3 4 5 6]),sets));
setOut2 = addTriangleSets(outputSetTriangle(polygon,index([2 3 4]),sets),outputSetSquare(polygon,index([1 4 5 6]),sets),outputSetTriangle(polygon,index([1 2 4]),sets));
setOut3 = addTriangleSets(outputSetSquare(polygon,index([2 3 4 5]),sets),outputSetTriangle(polygon,index([1 2 5]),sets),outputSetTriangle(polygon,index([1 5 6]),sets));
setOut4 = addTriangleSets(outputSetPentagon(polygon,index([2 3 4 5 6]),sets),outputSetTriangle(polygon,index([1 2 6]),sets));


setoutTemp = {setOut1, setOut2,setOut3,setOut4};
setOut.triangles = [];

for i = 1:length(setoutTemp)
    tempSet = setoutTemp{i};

    for j = 1:length(tempSet)
        if isempty(setOut(1).triangles)
            setOut(1).triangles = tempSet(j).triangles;
        else
            setOut(length(setOut)+1).triangles = tempSet(j).triangles;
        end
    end

end

end
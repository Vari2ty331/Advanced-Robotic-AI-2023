function outSets = addTriangleSets(set1,set2,setTriangle)

if nargin<3
    for i = 1:length(set1)
        for j = 1:length(set2)
            temp((i-1)*length(set2)+j).triangles = [set1(i).triangles ,set2(j).triangles];
        end
    end
else
    for i = 1:length(set1)
        for j = 1:length(set2)
            temp((i-1)*length(set2)+j).triangles = [set1(i).triangles, setTriangle.triangles,set2(j).triangles];
        end
    end

end

outSets = temp;

end
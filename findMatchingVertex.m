function matchingVertex = findMatchingVertex(polygon1,polygon2)

k = 1;

for i = 1:length(polygon1)
    for j = 1:length(polygon2)
        if polygon1(i,:) == polygon2(j,:)
            matchingVertex(k,:) = polygon1(i,:);
            k = k + 1;
        end
    end
end


end
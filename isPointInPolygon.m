function [bool,orientation] = isPointInPolygon(polygon,point)

for i = 1:length(polygon)-1
    polygonVectors(i,:) = polygon(i+1,:)-polygon(i,:);
end

polygonVectors(length(polygon),:) = polygon(1,:) - polygon(end,:);

centervectors = point - polygon;

crossProducts = cross(polygonVectors,centervectors,2);

crossProductsBoolean = crossProducts>0;

if all(crossProductsBoolean(:,3),'all')
    bool = true;
    orientation = 1;
elseif ~any(crossProductsBoolean(:,3),'all')
    bool = true;
    orientation = -1;
else
    bool = false;
    orientation = 0;
end
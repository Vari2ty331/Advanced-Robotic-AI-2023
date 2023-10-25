function outTriangle = outputSetTriangle(polygon,index,sets)

triangles.coords = polygon(index,:);
triangles.center = sum(polygon(index,:))/3;

if isempty(sets(end).triangles(end).coords)
    sets(end).triangles = triangles;
else
    sets = [sets, triangles];

end

outTriangle = sets;

% lastIdx = length(set(end).triangles);
% if ~isempty(set(end).triangles(1).coords)
%     set(end).triangles(lastIdx+1).coords = polygon(index,:);
% else
%     set(end).triangles(lastIdx).coords = polygon(index,:);
% end
% setOut = set;

end
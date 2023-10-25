function dist = getDistance(path)

dist = 0;

for i = 1:length(path)-1
    dist = dist + sqrt((path(i,:)-path(i+1,:))*(path(i,:)-path(i+1,:))');
end


end
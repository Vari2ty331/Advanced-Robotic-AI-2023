function vertices = createRandomPolygon(n)

sizeMultiplier = 1;
% offsetMargin = 0.3;
randPoints = rand(n,2);

angle = 2*pi/n;
for i = 1:length(linspace(0,2*pi,n))
    % offsetMargin = randi([0 300])*0.001;
    offsetMargin = 1 - cos(angle);
    refPoints(i,:) = [sizeMultiplier*cos((i-1)*angle) sizeMultiplier*sin((i-1)*angle)];
    polyPoints(i,:) = refPoints(i,:) + offsetMargin*randPoints(i,:);


end

vertices = polyPoints;

function plotPolygon(polygon)


figure()



plot(polygon([1:length(polygon),1],1),polygon([1:length(polygon),1],2),'-k',LineWidth=3);

end
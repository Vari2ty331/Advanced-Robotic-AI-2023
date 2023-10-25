function catalanNumber = findCatalanTriangles(polygons)


n = length(polygons);
catalanNumber = factorial(2*(n-2))/factorial(n-1)/factorial(n-2);



end
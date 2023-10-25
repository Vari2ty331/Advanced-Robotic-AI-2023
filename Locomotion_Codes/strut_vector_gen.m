function strut_vector = strut_vector_gen( tet_graph, strut )
% change node matrix to node vector (12 TET)

for i = 1:length(tet_graph)
    a = tet_graph(i,1);
    b = tet_graph(i,2);
    strut_vector(i,1) = strut(a,b);
end


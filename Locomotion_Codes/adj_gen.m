function adj = adj_gen( tet_graph )

for i = 1:length(tet_graph)
    num_node = max(max(tet_graph)); % number of node index
    for j = 1:num_node
        index_n1 = find(tet_graph(:,1) == j);
        index_n2 = find(tet_graph(:,2) == j);
        adj{j} = sort( [ tet_graph(index_n1, 2) ; tet_graph(index_n2, 1) ] )';
    end
   
end

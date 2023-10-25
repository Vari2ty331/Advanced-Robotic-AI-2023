function bool = checkNearNodeExtendable(tree,node)

input_tree = tree.T;

node_extendable = [];

node(end) =[];

for query_nodes = node'
    query_base_coords = input_tree(query_nodes).n;

    if checkFootMadePossibleWithMargin(query_base_coords,input_tree(end).center_point)
        node_extendable = [node_extendable true];

    else

        node_extendable = [node_extendable false];

    end
    
end

bool = any(node_extendable);

end
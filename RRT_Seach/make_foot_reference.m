function tree = make_foot_reference(tree,reset)

if nargin < 2
    reset = 0;
end

if reset == 0
    N = length(tree.T);
    tree.reference = [tree.reference; tree.T(N).foot];
else
    tree.reference = [];
    for i = 1:length(tree.T)
        tree.reference = [tree.reference; tree.T(i).foot];
    end
end
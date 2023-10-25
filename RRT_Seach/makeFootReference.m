function tree = makeFootReference(tree,reset)

if nargin < 2
    reset = 0;
end

if reset == 0
    N = length(tree.T);
    tree.reference = [tree.reference; tree.T(N).foot];
    last_idx = length(tree.reference);
    if length(tree.T(N).foot) == 9
        tree.pair = [
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx];
    else
        tree.pair = [last_idx-2 last_idx-2 last_idx-2;
                last_idx-1 last_idx-1 last_idx-1;
                last_idx last_idx last_idx];
    end
else
    tree.reference = [];
    tree.pair = [];
    for i = 1:length(tree.T)
        tree.reference = [tree.reference; tree.T(i).foot];
        last_idx = length(tree.reference);
        if length(tree.T(i).foot) == 9
            tree.pair = [tree.pair;
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-8 last_idx-7 last_idx-6;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx];
        elseif length(tree.T(i).foot) == 6
            tree.pair = [tree.pair;
                last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-5 last_idx-4 last_idx-3;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx;
            last_idx-2 last_idx-1 last_idx];
        else
            tree.pair = [tree.pair;
                last_idx-1 last_idx-1 last_idx-1;
                last_idx last_idx last_idx];
        end
    end
end
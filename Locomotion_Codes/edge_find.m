function index = edge_find( a,b,elist )
% find index of edge which compose of [a b] or [b a]
% returns 0 if there's no edge [a b] or [b a]

index = 0; % dummy index to check if it find index or not

for i = 1:length(elist)
    if ( elist(i,1) == a && elist(i,2) == b ) || ( elist(i,1) == b && elist(i,2) == a )
        index = i;
        break;
    end
end
    
end


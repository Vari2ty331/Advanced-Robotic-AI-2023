function clist = clist_gen(elist)
% generate edge collision check list from edge list

clist = [];
index = 1;

for i = 1:length(elist)
    for j = i+1:length(elist)
        if adj_check(elist,i,j) == 0
            clist(index,:) = [i j];
            index = index + 1;
        end
    end
end

end

function check = adj_check(elist,a,b)
% check if two nodes are in a edge or not

check = 0;
for i = 1:length(elist)
    if elist(a,1) == elist(b,1) || elist(a,1) == elist(b,2) || elist(a,2) == elist(b,1) || elist(a,2) == elist(b,2)
        check = 1;
    end
end

end
        






        
        
        
        
        
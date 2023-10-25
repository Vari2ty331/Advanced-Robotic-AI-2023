function l = lengths(elist, pos)
%lengths gives the length of each truss member
    if size(pos,1) > size(pos,2)
        pos = pos';
    end
    l = sqrt(sum((pos(:, elist(:,1)) - pos(:, elist(:,2))).^2 ))';
end
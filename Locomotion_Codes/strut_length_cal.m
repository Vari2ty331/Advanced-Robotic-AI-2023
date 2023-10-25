function strut = strut_length_cal(n)

% calculate length of strut
for i = 1:length(n.elist)
    a = n.elist(i,1);
    b = n.elist(i,2);
    strut(i,1) = norm( n.pos(a,:) - n.pos(b,:) );
end

end
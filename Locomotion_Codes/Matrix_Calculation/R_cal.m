function R = R_cal(x,n)

pos = reshape(x,size(n.pos));
elist = n.elist;
R = zeros(length(elist), length(x));

% calculate length of member
for i = 1:length(elist)
    a = elist(i,1);
    b = elist(i,2);
    L(i,1) = norm( pos(a,:) - pos(b,:) );
end

for k = 1:length(L)
    X = pos(:,1);
    Y = pos(:,2);
    Z = pos(:,3);

    i = elist(k,1);
    j = elist(k,2);

    R(k,i) = ( X(i) - X(j) ) / L(k);
    R(k,j) = ( X(j) - X(i) ) / L(k);
    R(k,length(pos) + i) = ( Y(i) - Y(j) ) / L(k);
    R(k,length(pos) + j) = ( Y(j) - Y(i) ) / L(k);
    R(k,2*length(pos) + i) = ( Z(i) - Z(j) ) / L(k);
    R(k,2*length(pos) + j) = ( Z(j) - Z(i) ) / L(k);
end
    
end
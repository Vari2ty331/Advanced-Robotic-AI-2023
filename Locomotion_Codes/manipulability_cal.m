function manip = manipulability_cal(elist,pos)

if size(pos,1) > size(pos,2)
    pos = pos'; % transpose to be compatible with function 'length_jacobian'
end

S = svd(length_jacobian(elist, pos));
S_mod = S(1:(length(elist)-6)); % J has at least 6 null space in 3D space
manip = min(S_mod)/max(S_mod);

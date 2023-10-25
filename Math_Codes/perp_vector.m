function u = perp_vector(P1,P2,P3)
% find vector u that;
% 1) parallel to the plain P1P2P3
% 2) perpendicular to P1P3
% vector u is the vector that pull point P3 to increase the angle P2P1P3

P1P2 = (P2 - P1) / norm(P2 - P1);
P1P3 = (P3 - P1) / norm(P3 - P1);

v = cross(P1P2,P1P3);
if norm(v) < 1e-10
    v = [0 0 0];
else
    v = v/norm(v);
end

u = cross(v,P1P3);
if norm(u) < 1e-10
    u = [0 0 0];
else
    u = u/norm(u);
end

end

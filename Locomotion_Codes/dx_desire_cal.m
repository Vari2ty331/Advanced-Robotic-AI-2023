function dx_desire = dx_desire_cal(n,x,data)

% reference: Linear Actuator Robots: Differential Kinematics, Controllability, and Algorithms for Locomotion and Shape Morphing (page 5th)
adj = adj_gen(n.elist);
dx_desire = zeros(length(n.pos),3);
pos = reshape(x,size(n.pos));
elist = n.elist;
angle_min = data.desired_angle_min;
L_nom = strut_length_cal(data.n_ini);

length_desire = 1;
angle_desire = 0;

%% nominal length function
if length_desire == 1
    
for i = 1:length(elist)
    k1 = elist(i,1);
    k2 = elist(i,2);
    L = pos(k2,:) - pos(k1,:);
    u = L/norm(L);
    temp_dx = (norm(L) - L_nom(i))*u;
    dx_desire(k1,:) = dx_desire(k1,:) + temp_dx;
    dx_desire(k2,:) = dx_desire(k2,:) - temp_dx;
end

end


%% min angle function
if angle_desire == 1

for i = 1:length(pos)
    for j = 1:length(adj{i})
        for k = j+1:length(adj{i})
            % node number7
            p1 = i;
            p2 = adj{i}(j);
            p3 = adj{i}(k);
            
            % node position
            x1 = pos(p1,1);     y1 = pos(p1,2);     z1 = pos(p1,3);
            x2 = pos(p2,1);     y2 = pos(p2,2);     z2 = pos(p2,3);
            x3 = pos(p3,1);     y3 = pos(p3,2);     z3 = pos(p3,3);
            
            P1 = [x1 y1 z1];
            P2 = [x2 y2 z2];
            P3 = [x3 y3 z3];
            
            P1P2 = (P2 - P1) / norm(P2 - P1);
            P1P3 = (P3 - P1) / norm(P3 - P1);
            angle = vector_angle(P1P2,P1P3);
            u1 = perp_vector(P1,P2,P3);
            u2 = perp_vector(P1,P3,P2);
            alpha = 0.02;
            
%             temp_dx1 = 1/abs(angle - angle_min)*u1*0.1;
%             temp_dx2 = 1/abs(angle - angle_min)*u2*0.1;

            temp_dx1 = (pi - angle) / angle_min*u1*alpha; 
            temp_dx2 = (pi - angle) / angle_min*u2*alpha;
            
            dx_desire(p3,:) = dx_desire(p3,:) + temp_dx1;
            dx_desire(p2,:) = dx_desire(p2,:) + temp_dx2;           
        end
    end
end

end

% normalization
for i = 1:length(pos)
    temp(i) = norm(dx_desire(i,:));
end
if max(temp) == 0
    scale = 0;
else
    scale = data.desired_vel_max/max(temp);
end

if scale < 1
    dx_desire = dx_desire*scale;
end
dx_desire = reshape(dx_desire,size(x));

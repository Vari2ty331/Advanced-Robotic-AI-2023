function T = foot_gen(T,L_nom,Ground) 


if nargin < 2
    L_nom = 1;
end

if T.Parent_Index == 0
    p1 = 1;
    p2 = 2;
    p3 = [1 2 3];
    p3([p1 p2]) = [];   
    T.foot(p3,:) = foot_cal(T,L_nom,p1,p2,p3,Ground);
    
    p1 = 2;
    p2 = 3;
    p3 = [1 2 3];
    p3([p1 p2]) = [];   
    T.foot(p3,:) = foot_cal(T,L_nom,p1,p2,p3,Ground);
    
    p1 = 3;
    p2 = 1;
    p3 = [1 2 3];
    p3([p1 p2]) = [];   
    T.foot(p3,:) = foot_cal(T,L_nom,p1,p2,p3,Ground);
else
    p1 = 2;
    p2 = 3;
    p3 = [1 2 3];
    p3([p1 p2]) = [];   
    T.foot(p3,:) = foot_cal(T,L_nom,p1,p2,p3,Ground);
    
    p1 = 3;
    p2 = 1;
    p3 = [1 2 3];
    p3([p1 p2]) = [];   
    T.foot(p3,:) = foot_cal(T,L_nom,p1,p2,p3,Ground);
end
    
   
function foot = foot_cal(T,L_nom,p1,p2,p3,Ground)

n = T.n;

x1 = n(p1,1);   x2 = n(p2,1);   x3 = n(p3,1);
y1 = n(p1,2);   y2 = n(p2,2);   y3 = n(p3,2);
z1 = n(p1,3);   z2 = n(p2,3);   z3 = n(p3,3);

xc = ( x1 + x2 ) / 2;
yc = ( y1 + y2 ) / 2;
zc = ( z1 + z2 ) / 2;

h = sqrt(abs(L_nom^2 - ( norm([x2-x1 y2-y1])/2 )^2) );
a = y2 - y1;
b = x1 - x2;
c = (x2 - x1)*y1 + (y1 - y2)*x1;

line_fun = @(x,y) a*x + b*y + c; % line equation which coincide with (xc,yc) and perpendicular to the line connecting (x1,y1), (x2,y2)

foot_x = xc + h*a/norm([a b]);
foot_y = yc + h*b/norm([a b]);
foot_z = zc;

if line_fun(foot_x,foot_y)*line_fun(x3,y3) > 0
    foot_x = xc - h*a/norm([a b]);
    foot_y = yc - h*b/norm([a b]);
    foot_z = zc;
end

foot_init = [foot_x; foot_y; foot_z;];

% Determine the direction of rotating the front node.
% Lift the foot pi/4 rad first.
% If the rotation does not lift it, change the direction of rotation.
% (The direction of rotation line is opposite)

foot = foot_init;

% rotation_sign = 1;
% foot_rotated = Rotating_Line(foot_init, [n(p1,1); n(p1,2); n(p1,3);], [n(p2,1); n(p2,2); n(p2,3);], pi/4);
% if foot_rotated(3) < foot_z
%     foot_rotated = Rotating_Line(foot_init, [n(p1,1); n(p1,2); n(p1,3);], [n(p2,1); n(p2,2); n(p2,3);], -pi/4);
%     rotation_sign = -1;
% end
% 
% [foot_collcheck, foot_distance] = Ground_Collision(Ground, foot_rotated.');
% 
% % Measure the distance between the foot candidate and the ground.
% % Move foot close to the ground briefly use the distance by rotation.
% foot_distance = min(foot_distance);
% 
% foot_rotated = Rotating_Line(foot_rotated, [n(p1,1); n(p1,2); n(p1,3);], [n(p2,1); n(p2,2); n(p2,3);], -rotation_sign * h/foot_distance);
% foot_collcheck = Ground_Collision(Ground, foot_rotated.');
% 
% % Check if the foot candidate collide with the ground by previous rotation.
% % If the foot candidate collide with the ground, keep lifting the foot by
% % rotation until it is not colliding with the ground.
% while isempty(find(foot_collcheck)) ~= 1
%     foot_rotated = Rotating_Line(foot_rotated, [n(p1,1); n(p1,2); n(p1,3);], [n(p2,1); n(p2,2); n(p2,3);], rotation_sign * 0.1);
%     foot_collcheck = Ground_Collision(Ground, foot_rotated.');
% end
% 
% % After set the foot just above the ground, slowly move the foot down until
% % collision, and confirm the contacting point.
% while isempty(find(foot_collcheck)) == 1
%     foot_collcheck = Ground_Collision(Ground, foot_rotated.');
%     foot_rotated = Rotating_Line(foot_rotated, [n(p1,1); n(p1,2); n(p1,3);], [n(p2,1); n(p2,2); n(p2,3);], -rotation_sign * 0.005);
% end
% 
% foot = foot_rotated;
% 
% 
% 
% 
% function P_Rotated = Rotating_Line(H, P1, P2, theta)
% % A code for rotation a point 'H' w.r.t. the line between 'P1' and 'P2'
%     Rot_Axis = P2 - P1;
%     x0 = P1(1);
%     y0 = P1(2);
%     z0 = P1(3);
% 
%     Rot_Axis = Rot_Axis / norm(Rot_Axis);
% 
%     H_zero = H + [-x0; -y0; -z0;];
% 
%     H_zero_Q = quaternion(0, H_zero(1), H_zero(2), H_zero(3));
% 
%     Rotation_Q = quaternion(cos(theta/2), Rot_Axis(1)*sin(theta/2), Rot_Axis(2)*sin(theta/2), Rot_Axis(3)*sin(theta/2));
% 
%     H_Rot_Q = Rotation_Q * H_zero_Q * conj(Rotation_Q);
%     [~, H_Rot(1), H_Rot(2), H_Rot(3)] = parts(H_Rot_Q);
% 
%     P_Rotated = H_Rot.' + [x0; y0; z0;];
%     
% % end


function foot = foot_cal_pol(T,L_nom,p1,p2,p3,Ground)

n = T.n;

x1 = n(p1,1);   x2 = n(p2,1);   x3 = n(p3,1);
y1 = n(p1,2);   y2 = n(p2,2);   y3 = n(p3,2);
z1 = n(p1,3);   z2 = n(p2,3);   z3 = n(p3,3);

xc = ( x1 + x2 ) / 2;
yc = ( y1 + y2 ) / 2;
zc = ( z1 + z2 ) / 2;

h = sqrt(abs(L_nom^2 - ( norm([x2-x1 y2-y1])/2 )^2) );
a = y2 - y1;
b = x1 - x2;
c = (x2 - x1)*y1 + (y1 - y2)*x1;

line_fun = @(x,y) a*x + b*y + c; % line equation which coincide with (xc,yc) and perpendicular to the line connecting (x1,y1), (x2,y2)

foot_x = xc + h*a/norm([a b]);
foot_y = yc + h*b/norm([a b]);
foot_z = zc;

if line_fun(foot_x,foot_y)*line_fun(x3,y3) > 0
    foot_x = xc - h*a/norm([a b]);
    foot_y = yc - h*b/norm([a b]);
    foot_z = zc;
end

foot_init = [foot_x; foot_y; foot_z;];

% Determine the direction of rotating the front node.
% Lift the foot pi/4 rad first.
% If the rotation does not lift it, change the direction of rotation.
% (The direction of rotation line is opposite)

foot = foot_init;
function P = truss_force_cal(elist,pos,fixed_node,force_index)
% force index
% 1: force at tensioner
% 2: force at drive
% 3: force applied to tensioner (related to tensile actuation strength)
% 4: force applied to drive (related to compressive actuation strength)

[L, mass_sphere, mass_member, com_member] = com_member_cal(elist,pos); % Calculate L, mass and com of members and sphere

if nargin < 4
    force_index = 1:4;
end

if size(pos,1) > size(pos,2)
    pos = pos';
end

truss_load = zeros( size(pos) );
truss_load(3,:) = -mass_sphere; % set mass of node

A = zeros( 6*length(elist) + 3*length(pos), 6*length(elist) + 3*length(fixed_node) );
B = zeros( 6*length(elist) + 3*length(pos), 1 );


%% Member equation (from FBD of member)

for i = 1:length(elist)
    F1x = 6*i - 5;
    F1y = 6*i - 4;
    F1z = 6*i - 3;
    F2x = 6*i - 2;
    F2y = 6*i - 1;
    F2z = 6*i;
    
    Fx_line = 6*i - 5;
    Fy_line = 6*i - 4;
    Fz_line = 6*i - 3;
    Mx_line = 6*i - 2;
    My_line = 6*i - 1;
    Mz_line = 6*i;
    
    pos_vec = pos(:,elist(i,2)) - pos(:,elist(i,1));
    Lx = pos_vec(1);
    Ly = pos_vec(2);
    Lz = pos_vec(3);
    
    % force equation
    A(Fx_line,[F1x F2x]) = 1;
    A(Fy_line,[F1y F2y]) = 1;
    A(Fz_line,[F1z F2z]) = 1;
%     B(Fz_line,1) = -sum(-mass_member{i}) - truss_load(3,elist(i,1)) - truss_load(3,elist(i,2)); % give (-) to mass_member due to gravity force 
    B(Fz_line,1) = -sum(-mass_member{i}); % give (-) to mass_member due to gravity force 

    % Must discuss whether the mass_sphere should be considered
    
    % moment equation
    A(Mx_line,F2z) = Ly;
    A(Mx_line,F2y) = -Lz;
    B(Mx_line,1) = -dot( -mass_member{i} , Ly*(com_member{i}/L(i)) ); % give (-) to mass_member due to gravity force
    
    A(My_line,F2x) = Lz;
    A(My_line,F2z) = -Lx;
    B(My_line,1) = dot( -mass_member{i} , Lx*(com_member{i}/L(i)) ); % give (-) to mass_member due to gravity force
    
    A(Mz_line,F2y) = Lx;
    A(Mz_line,F2x) = -Ly;
end

%% Joint equation (from FBD of joint)
f_index = 1;
for i = 1:length(pos)
    adj_member = find(elist' == i);
    
    Fx_line = 6*length(elist) + 3*i - 2;
    Fy_line = 6*length(elist) + 3*i - 1;
    Fz_line = 6*length(elist) + 3*i;
    
    Fx = 3*adj_member' - 2;
    Fy = 3*adj_member' - 1;
    Fz = 3*adj_member';
    
    % force_equation
    A(Fx_line, Fx) = -1;
    A(Fy_line, Fy) = -1;
    A(Fz_line, Fz) = -1;
    
    if isempty(find(fixed_node == i,1)) == 0
        Rx = 6*length(elist) + 3*f_index - 2;
        Ry = 6*length(elist) + 3*f_index - 1;
        Rz = 6*length(elist) + 3*f_index;
                 
        A(Fx_line,Rx) = 1;
        A(Fy_line,Ry) = 1;
        A(Fz_line,Rz) = 1;
        
        f_index = f_index + 1;
    end

    B(Fx_line) = -truss_load(1,i);
    B(Fy_line) = -truss_load(2,i);
    B(Fz_line) = -truss_load(3,i);

end

%% Deflection equation
% Having fixed point constrain at the bottom nodes, the truss structure is statically indetermined. 
% So deflection eqauation was added for the members which have fixed point at both ends.

for i = 1:length(elist)
    if isempty(find(fixed_node == elist(i,1),1)) == 0 && isempty(find(fixed_node == elist(i,2),1)) == 0   
        
        F1x = 6*i - 5;
        F1y = 6*i - 4;
        F1z = 6*i - 3;
        F2x = 6*i - 2;
        F2y = 6*i - 1;
        F2z = 6*i;
        
        temp = pos(:,elist(i,1)) - pos(:,elist(i,2));
        L = norm(temp);
        Lx = temp(1)/2;
        Ly = temp(2)/2;
        Lz = temp(3)/2;
        
        row = size(A,1);
        A(row+1,F1x) = 1;
        A(row+2,F1y) = 1;
        A(row+3,F1z) = 1;
        A(row+1,F2x) = -1;
        A(row+2,F2y) = -1;
        A(row+3,F2z) = -1;
        B(row+1:row+3,1) = 0;      
    end
end
       

% X = A\B;
X = pinv(A)*B;
F_ori = X(1:6*length(elist));
R_ori = X(6*length(elist)+1:end);

F = reshape(F_ori,6,length(elist));
R = reshape(R_ori,3,length(fixed_node));

%% Axial force cal
for i = 1:length(elist)
    pos_vec = pos(:,elist(i,2)) - pos(:,elist(i,1));
    u1 = pos_vec/norm(pos_vec);
    u2 = -u1;
    
    P(i,1) = dot(u1,F(1:3,i));
    P(i,2) = dot(u2,F(4:6,i));
    P(i,3) = P(i,1) + ( P(i,2) - P(i,1) )*( sum(mass_member{i}(1)) / sum(mass_member{i}) );
    P(i,4) = P(i,2) + ( P(i,1) - P(i,2) )*( sum(mass_member{i}(3)) / sum(mass_member{i}) );
        
    v1 = F(1:3,i) - P(i,1)*u1; 
    v2 = F(4:6,i) - P(i,2)*u2;
    
    V(i,1) = norm(v1);
    V(i,2) = norm(v2);
end

%% Output value
P = P(:,force_index);

%% Debugging
% accuracy = norm(A*X-B);

% rank_check = length(X) - rank(A);
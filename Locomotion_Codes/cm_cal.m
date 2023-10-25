function [x0_cm, x_cm, dx_cm] = cm_cal(elist,x0,dx)

if nargin < 3
    dx = zeros(size(x0));
end

x = x0 + dx;

% Calculate com of x0
pos = reshape(x0,[length(x0)/3, 3]);
[L, mass_sphere, mass_member, com_member] = com_member_cal(elist,pos);
temp_cm = 0;

for i = 1:length(elist)    
    u = ( pos(elist(i,2),:) - pos(elist(i,1),:) ) / L(i);
    for j = 1:length(com_member{i})
        com_member_pos = pos(elist(i,1),:)' + u'*com_member{i}(j);
        temp_cm = temp_cm + mass_member{i}(j)*com_member_pos;
    end
end

for i = 1:length(pos)
    temp_cm = temp_cm + pos(i,:)'*mass_sphere(i);
end

sum_mass_member = 0;
for i = 1:length(elist)
    sum_mass_member = sum_mass_member + sum(mass_member{i});
end

x0_cm = temp_cm / ( sum_mass_member + sum(mass_sphere) );


% Calculate com of x
pos = reshape(x,[length(x)/3, 3]);
[L, mass_sphere, mass_member, com_member] = com_member_cal(elist,pos);
temp_cm = 0;

for i = 1:length(elist)    
    u = ( pos(elist(i,2),:) - pos(elist(i,1),:) ) / L(i);
    for j = 1:length(com_member{i})
        com_member_pos = pos(elist(i,1),:)' + u'*com_member{i}(j);
        temp_cm = temp_cm + mass_member{i}(j)*com_member_pos;
    end
end

for i = 1:length(pos)
    temp_cm = temp_cm + pos(i,:)'*mass_sphere(i);
end

x_cm = temp_cm / ( sum_mass_member + sum(mass_sphere) );

% Calculate dx_cm
dx_cm = x_cm - x0_cm;

end
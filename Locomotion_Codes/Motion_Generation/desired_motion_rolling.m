function [dx_temp, equal_const, inequal_const, constraint_flag] = desired_motion_rolling(D,n,R,C,M,x,dt,rotate_node,front_node,back_node,pos_foot_node,data)

%% Setting

constraint_flag = 0;
dx_cm = [0 0 0]';
fixed_member = data.fixed_member{end};
Rf = R(fixed_member,:);

vel_max = data.desired_vel_max;
acc_max = data.desired_acc_max;

x0 = data.x(end,:)';
% dx0 = data.dx(end,:)';
dx0 = zeros(size(x0));
elist = n.elist;

% Variable for acc. constraint
dx_pre = data.dx(end,:)';
x_pre = data.x(end-1,:);
R_pre = R_cal(x_pre,data.n_ini);

%% Objective Function

% fun = @(dx) min_altitude_cal(x,dx,dt,rotate_node,front_node,back_node);
fun = @(dx) min_foot_distance_cal(x,dx,front_node,back_node,pos_foot_node);

%% Optimization

% Nonlinear constraint for center of mass (Set cm_cal_nonlcon properly)
A = [R;-R;-D;R;-R];
B = [vel_max*ones(size(R,1),1) ; vel_max*ones(size(R,1),1) ; -1e-6*ones(size(D,1),1) ; R_pre*dx_pre + acc_max ; -R_pre*dx_pre + acc_max];
Aeq = [C;Rf];
Beq = [zeros(size(C,1),1);zeros(size(Rf,1),1)];
nonlcon = @(dx) cm_cal_nonlcon(data,x0,dx,dx_cm);
options = optimset('Display','off','MaxFunEvals',inf,'Algorithm','interior-point','TolCon',1e-14);  % MaxFunEvals default: 3000
% fmincon algorithms: 'active-set', 'trust-region-reflective', 'interior-point', 'interior-point-convex',
% 'interior-point-legacy', 'levenberg-marquardt', 'trust-region-dogleg', or 'sqp'

dx = fmincon(fun,dx0,A,B,Aeq,Beq,[],[],nonlcon,options);

% % Nonlinear constraint for center of mass and fixed member (Set cm_cal_nonlcon properly)
% A = [R;-R;-D;R;-R];
% B = [vel_max*ones(size(R,1),1) ; vel_max*ones(size(R,1),1) ; -1e-6*ones(size(D,1),1) ; R_pre*dx_pre + acc_max ; -R_pre*dx_pre + acc_max];
% Aeq = C;
% Beq = zeros(size(C,1),1);
% nonlcon = @(dx) cm_cal_nonlcon(data,x0,dx,dx_cm);
% options = optimset('Display','off','MaxFunEvals',inf,'Algorithm','interior-point','TolCon',1e-14);  % MaxFunEvals default: 3000
% % fmincon algorithms: 'active-set', 'trust-region-reflective', 'interior-point', 'interior-point-convex',
% % 'interior-point-legacy', 'levenberg-marquardt', 'trust-region-dogleg', or 'sqp'

% dx = fmincon(fun,dx0,A,B,Aeq,Beq,[],[],nonlcon,options);

% use M matrix
A = [R;-R;-D;R;-R];
B = [vel_max*ones(size(R,1),1) ; vel_max*ones(size(R,1),1) ; -1e-6*ones(size(D,1),1) ; R_pre*dx_pre + acc_max ; -R_pre*dx_pre + acc_max];
Aeq = [C;M;Rf];
Beq = [zeros(size(C,1),1);dx_cm;zeros(size(Rf,1),1)];
options = optimset('Display','off','MaxFunEvals',3000,'Algorithm','interior-point','Tolfun',1e-14);  % MaxFunEvals default: 3000
% fmincon algorithms: 'active-set', 'trust-region-reflective', 'interior-point', 'interior-point-convex',
% 'interior-point-legacy', 'levenberg-marquardt', 'trust-region-dogleg', or 'sqp'

dx = fmincon(fun,dx0,A,B,Aeq,Beq,[],[],[],options);



equal_const = norm(Aeq*dx - Beq);
inequal_plus = ( (A*dx - B) + abs(A*dx - B) )/2; % inequality values with plus sign

inequal_const = norm(inequal_plus);

if equal_const > 1e-3 
    fprintf('fmincon does not satisfy equaility constraint Aeq*x - Beq = %d\n',equal_const)
    constraint_flag = 1;
end

if inequal_const > 1e-3 
    fprintf('fmincon does not satisfy inequaility constraint A*x - B = %d\n',inequal_const)
    constraint_flag = 1;
end

% % for debugging
% global g_inequal
% temp_inequal = A*x - B;
% g_inequal(size(data.x,1)+1,:) = temp_inequal(1:32);
if ~isempty(fixed_member)
    fprintf(['fixed_member: ' num2str(fixed_member) '     Rfdx: ' num2str(norm(Rf*dx))  ' \n'])
end

dx_temp = dx;

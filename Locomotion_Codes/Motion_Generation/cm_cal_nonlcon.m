function [c, ceq] = cm_cal_nonlcon(data,x0,dx,dx_cm_desire)

% variable definition
elist = data.n_ini.elist;
fixed_member = data.fixed_member{end};

% center of mass constraint
[~, ~, dx_cm] = cm_cal(elist,x0,dx);
ceq1 = dx_cm(1:2) - dx_cm_desire(1:2);

% % fixed_member constraint
% pos0 = reshape(x0,size(data.n_ini.pos));
% pos = reshape(x0+dx,size(data.n_ini.pos));
% 
% L0 = lengths(elist,pos0);
% L = lengths(elist,pos);
% 
% ceq2 = [];
% for i = 1:length(fixed_member)   
%     ceq2 = [ ceq2 ; L0(fixed_member(i)) - L(fixed_member(i)) ];
% end

c = [];
% ceq = [ceq1 ; ceq2]; % only consider x,y coordinate
ceq = ceq1;
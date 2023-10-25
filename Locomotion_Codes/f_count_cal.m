function [f_count, data, fix_flag] = f_count_cal(data,n,P,f_count)
% f_count represent how many time the edge exceed the force constraint during optimization. 
fix_flag = 0; % index to check if the set of fixed member is changed

%% Load variables
elist = n.elist;

max_compressive_force = data.max_compressive_force;
max_tensile_force = data.max_tensile_force;

fixed_member = data.fixed_member{end};
max_f_count = data.max_f_count;

%% Calculate f_count and define fixed member

for i = 1:length(elist)
    if isempty(find(fixed_member == i, 1)) % if the member is not fixed member
        if P(i,3) > max_compressive_force %|| P_t(i) > max_tensile_force % exceed strength
            f_count(i) = f_count(i) + 1;
            if f_count(i) >= max_f_count
                fixed_member = [fixed_member i];
                fix_flag = fix_flag + 1; % set of fixed member changed
                sort(fixed_member);
                f_count(i) = 0;
            end
        end
    else % if the member is fixed member
        if P(i,3) <= max_compressive_force %&& P_t(i) <= max_tensile_force % within strength
            fixed_member(fixed_member == i) = [];
%             fix_flag = fix_flag + 1; % set of fixed member changed. Warning: possible to be infinit loop.
        end
    end
end % for end

data.fixed_member{end} = fixed_member;

%% Setting for non fixed member case
% % also set strength_check = 1 in constraint_gen code
% data.fixed_member{end} = [];
% fix_flag = 0;
            
        
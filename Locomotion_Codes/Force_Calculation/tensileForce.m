%% Calculate tensile force.  It is used in calc_comp_tens_force.m file.

function [P_t] = tensileForce(P, zipper_index)
    for i = 1:length(P)
        temp_P = P(i,zipper_index);
        if temp_P > 0 
            temp_P = 0;
        end
        P_t(i,1) = abs(temp_P); % tensile force
    end 
end

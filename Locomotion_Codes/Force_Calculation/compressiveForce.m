%% Calculate compressive force. It is used in calc_comp_tens_force.m file.

function [P_c] = compressiveForce(P, zipper_index)
    for i = 1:length(P)
        temp_P = P(i,zipper_index);
        if temp_P < 0 
            temp_P = 0;
        end
        P_c(i,1) = temp_P; % compressive force
    end
end
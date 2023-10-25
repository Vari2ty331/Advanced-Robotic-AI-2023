%% Calculate compressive force and tensile force. The pervious code was "comp_tens_force_cal.m". The change is for loop to compressive and tensile function.


function [P_c, P_t, Reaction, Req_Fric, Normal_Force] = calc_comp_tens_force(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff)

[P, Reaction, Req_Fric, Normal_Force] = truss_force_cal(elist,pos,fixed_node,surface_normal_vector,MaxStaticCoeff);

P_c = compressiveForce(P,4);
P_t = tensileForce(P, 3);
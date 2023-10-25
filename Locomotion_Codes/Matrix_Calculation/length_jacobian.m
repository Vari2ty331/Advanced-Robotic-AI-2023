function [ J ] = length_jacobian( elist, pos )
%length_jacobian gives the jacobian L_dot = J(x)*x_dot
%   See is_inf_rigid for more details, 
%   J is a scaled version of the rigidity matrix

L = lengths(elist, pos);

% scale each row of the rigidity matrix by the corresponding member length
J = rigidity_matrix(elist, pos)./L; 

assert(~any(isnan(J(:))))

end

function [dihedral_angle, dihedral_angle_connection] = dihedral_angle_sub_cal(dihedral_angle_connection,elist,pos)
   
    if size(pos,1) > size(pos,2)
        pos = pos';
    end
         
    dihedral_angle = zeros(size(dihedral_angle_connection,1),1);

    for i = 1:size(dihedral_angle_connection,1)
        edge_1 = dihedral_angle_connection(i,1);
        edge_inter = dihedral_angle_connection(i,2);
        edge_2 = dihedral_angle_connection(i,3);
        if any(elist(edge_inter,1) == elist(edge_1,:))
            p_0 = elist(edge_inter,1);
            p_inter = elist(edge_inter,2);
            p_1 = elist(edge_1,elist(edge_1,:) ~= p_0);
            p_2 = elist(edge_2,elist(edge_2,:) ~= p_0);
            
        else % any(elist(edge_inter,2) == elist(edge_1,:))
            p_0 = elist(edge_inter,2);
            p_inter = elist(edge_inter,1);
            p_1 = elist(edge_1,elist(edge_1,:) ~= p_0);
            p_2 = elist(edge_2,elist(edge_2,:) ~= p_0);
        end
        
        vector_1 = pos(:,p_1) - pos(:,p_0); 
        vector_inter = pos(:,p_inter) - pos(:,p_0);
        vector_2 = pos(:,p_2) - pos(:,p_0);
        alpha = vector_inter' * vector_1 / norm(vector_inter)^2;
        beta = vector_inter' * vector_2 / norm(vector_inter)^2;
        vector_alpha = vector_1 - alpha * vector_inter;
        vector_beta = vector_2 - beta * vector_inter;
        
        cos_theta = vector_alpha'*vector_beta / (norm(vector_alpha) * norm(vector_beta));
        
        % Numerical error handling
        if cos_theta > 1 && cos_theta < 1+1e-5
            cos_theta = 1;
        elseif cos_theta < -1 && cos_theta > -1-1e-5
            cos_theta = -1;
        end
        
        dihedral_angle(i) = acos(cos_theta);
    end

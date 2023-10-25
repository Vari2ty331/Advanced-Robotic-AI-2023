function dihedral_angle_connection = dihedral_angle_connection_gen(link_connection)
% calculate dihedral angles

    % link_connection must be column vector
    if size(link_connection,1) < size(link_connection,2)
        link_connection = link_connection';
    end
    
    dihedral_angle_connection = [];
    
%% Step 1. Find member set connected by linkages
    for i = 1:size(link_connection,1)  
        % check whether link connection is valid
        if any(link_connection{i}(:,1) == link_connection{i}(:,2))
            error('Some linkage connect itself (some link_connection is [a a]).')
        end
        if ~length(unique(link_connection{i}(:,1))) == length(link_connection{i}(:,1))
            error('There are overlapped (starting) link_connection index ([a b;~ ~;a c]).')
        end
        
%         link_connection{i} = sort(link_connection{i},2);
        
        for j = 1:size(link_connection{i},1)-1
            for k = (j+1):size(link_connection{i},1)
                if any(link_connection{i}(k,1) == link_connection{i}(j,:)) && ~any(link_connection{i}(j,:) == 0) && ~any(link_connection{i}(k,:) == 0)
                    if link_connection{i}(k,1) == link_connection{i}(j,1)
                        dihedral_angle_connection = [dihedral_angle_connection;link_connection{i}(j,2) link_connection{i}(j,1) link_connection{i}(k,2)];
                    else % link_connection{i}(k,1) == link_connection{i}(j,2)
                        dihedral_angle_connection = [dihedral_angle_connection;link_connection{i}(j,1) link_connection{i}(j,2) link_connection{i}(k,2)];
                    end
                elseif any(link_connection{i}(k,2) == link_connection{i}(j,:)) && ~any(link_connection{i}(j,:) == 0) && ~any(link_connection{i}(k,:) == 0)
                    if link_connection{i}(k,2) == link_connection{i}(j,1)
                        dihedral_angle_connection = [dihedral_angle_connection;link_connection{i}(j,2) link_connection{i}(j,1) link_connection{i}(k,1)];
                    else % link_connection{i}(k,2) == link_connection{i}(j,2)
                        dihedral_angle_connection = [dihedral_angle_connection;link_connection{i}(j,1) link_connection{i}(j,2) link_connection{i}(k,1)];
                    end
                end
            end
        end
    end

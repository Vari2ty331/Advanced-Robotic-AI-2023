function Lin_Transform_A = MPLinearTransform(x_ini, x_fin, x_tilda_ini, x_tilda_fin, x_length)

    % All size of position vector in this function is x_length/3 * 3
    % x_ini: initial position of motion primitive.
    % x_fin: final position of motion primitive.
    % x_tilda_ini: initial position of the previous support polygon.
    % x_tilda_fin: final position of the next support polygon.
    
    % Change the input shape if the shape is one line vector
    
    if ~isequal(size(x_ini), [x_length/3 3])
        x_ini = reshape(x_ini, [x_length/3 3]);
    end
    if ~isequal(size(x_fin), [x_length/3 3])
        x_fin = reshape(x_fin, [x_length/3 3]);
    end
    if ~isequal(size(x_tilda_ini), [x_length/3 3])
        x_tilda_ini = reshape(x_tilda_ini, [x_length/3 3]);
    end
    if ~isequal(size(x_tilda_fin), [x_length/3 3])
        x_tilda_fin = reshape(x_tilda_fin, [x_length/3 3]);
    end
    

    Lin_Transform_A = zeros(x_length,x_length);                                 % Linear transformation A to transform motion primitive on distorted support polygon
    Lin_Transform_Tol = 1e-3;                                                   % Preventing singularity of not-moving nodes.

    for k = 1 : x_length/3
        if norm(x_fin(k,:) - x_ini(k,:)) >= Lin_Transform_Tol
            Lin_Transform_A_Part{k} = eye(3) + (((x_tilda_fin(k,:).' - x_tilda_ini(k,:).') - (x_fin(k,:).' - x_ini(k,:).')) * (x_fin(k,:).' - x_ini(k,:).').') / (norm(x_fin(k,:) - x_ini(k,:)))^2;
        else
            Lin_Transform_A_Part{k} = eye(3);
        end
    end

    for k = 1 : x_length/3
        Lin_Transform_A(k, k) =  Lin_Transform_A_Part{k}(1,1);
        Lin_Transform_A(k, x_length/3 + k) =  Lin_Transform_A_Part{k}(1,2);
        Lin_Transform_A(k, 2 * x_length/3 + k) =  Lin_Transform_A_Part{k}(1,3);

        Lin_Transform_A(x_length/3 + k, k) =  Lin_Transform_A_Part{k}(2,1);
        Lin_Transform_A(x_length/3 + k, x_length/3 + k) =  Lin_Transform_A_Part{k}(2,2);
        Lin_Transform_A(x_length/3 + k, 2 * x_length/3 + k) =  Lin_Transform_A_Part{k}(2,3);

        Lin_Transform_A(2 * x_length/3 + k, k) =  Lin_Transform_A_Part{k}(3,1);
        Lin_Transform_A(2 * x_length/3 + k, x_length/3 + k) =  Lin_Transform_A_Part{k}(3,2);
        Lin_Transform_A(2 * x_length/3 + k, 2 * x_length/3 + k) =  Lin_Transform_A_Part{k}(3,3);
    end



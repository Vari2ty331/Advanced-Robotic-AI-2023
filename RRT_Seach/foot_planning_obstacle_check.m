
% obs_index = [1 3 2 3 ; 1 4 2 4 ; 1 3 1 4 ; 2 3 2 4]; 

index = 1;

for i = 1:length(obstacle) 
    for j = 1:2
        for k = 1:length(obstacle{i})-1
            edge_set{index,1} = [polygon([j 3],:)]; %   
            edge_set{index,1} = [edge_set{index,1} ; obstacle{i}(k,:) ; obstacle{i}(k+1,:)];
            index = index + 1;
        end
        k = length(obstacle{i});
        edge_set{index,1} = [polygon([j 3],:)]; %   
        edge_set{index,1} = [edge_set{index,1} ; obstacle{i}(k,:) ; obstacle{i}(1,:)];
        index = index + 1;
    end
end


index = 1;
dist_con = [];

for k = 1:length(edge_set)

    % %%%%% line to line collision check %%%%%% 
    % node number
    x1 = edge_set{k}(1,1);     y1 = edge_set{k}(1,2);     z1 = 0;
    x2 = edge_set{k}(2,1);     y2 = edge_set{k}(2,2);     z2 = 0;
    x3 = edge_set{k}(3,1);     y3 = edge_set{k}(3,2);     z3 = 0;
    x4 = edge_set{k}(4,1);     y4 = edge_set{k}(4,2);     z4 = 0;
    
    t1 = (x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + x1*x3*y3*y4 + y2*x1*x3*y3 - x1*x3*y4^2 - y2*x1*x3*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + x1*x3*z3*z4 + z2*x1*x3*z3 - x1*x3*z4^2 - z2*x1*x3*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - x1*x4*y3^2 + x1*x4*y3*y4 - y2*x1*x4*y3 + y2*x1*x4*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - x1*x4*z3^2 + x1*x4*z3*z4 - z2*x1*x4*z3 + z2*x1*x4*z4 - x2*x1*y3^2 + 2*x2*x1*y3*y4 - x2*x1*y4^2 - x2*x1*z3^2 + 2*x2*x1*z3*z4 - x2*x1*z4^2 + x3^2*y1^2 - x3^2*y1*y4 - y2*x3^2*y1 + y2*x3^2*y4 + x3^2*z1^2 - x3^2*z1*z4 - z2*x3^2*z1 + z2*x3^2*z4 - 2*x3*x4*y1^2 + x3*x4*y1*y3 + x3*x4*y1*y4 + 2*y2*x3*x4*y1 - y2*x3*x4*y3 - y2*x3*x4*y4 - 2*x3*x4*z1^2 + x3*x4*z1*z3 + x3*x4*z1*z4 + 2*z2*x3*x4*z1 - z2*x3*x4*z3 - z2*x3*x4*z4 + x2*x3*y1*y3 - x2*x3*y1*y4 - x2*x3*y3*y4 + x2*x3*y4^2 + x2*x3*z1*z3 - x2*x3*z1*z4 - x2*x3*z3*z4 + x2*x3*z4^2 + x4^2*y1^2 - x4^2*y1*y3 - y2*x4^2*y1 + y2*x4^2*y3 + x4^2*z1^2 - x4^2*z1*z3 - z2*x4^2*z1 + z2*x4^2*z3 - x2*x4*y1*y3 + x2*x4*y1*y4 + x2*x4*y3^2 - x2*x4*y3*y4 - x2*x4*z1*z3 + x2*x4*z1*z4 + x2*x4*z3^2 - x2*x4*z3*z4 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + y1*y3*z3*z4 + z2*y1*y3*z3 - y1*y3*z4^2 - z2*y1*y3*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - y1*y4*z3^2 + y1*y4*z3*z4 - z2*y1*y4*z3 + z2*y1*y4*z4 - y2*y1*z3^2 + 2*y2*y1*z3*z4 - y2*y1*z4^2 + y3^2*z1^2 - y3^2*z1*z4 - z2*y3^2*z1 + z2*y3^2*z4 - 2*y3*y4*z1^2 + y3*y4*z1*z3 + y3*y4*z1*z4 + 2*z2*y3*y4*z1 - z2*y3*y4*z3 - z2*y3*y4*z4 + y2*y3*z1*z3 - y2*y3*z1*z4 - y2*y3*z3*z4 + y2*y3*z4^2 + y4^2*z1^2 - y4^2*z1*z3 - z2*y4^2*z1 + z2*y4^2*z3 - y2*y4*z1*z3 + y2*y4*z1*z4 + y2*y4*z3^2 - y2*y4*z3*z4)/(x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x2*y3^2 + 4*x1*x2*y3*y4 - 2*x1*x2*y4^2 - 2*x1*x2*z3^2 + 4*x1*x2*z3*z4 - 2*x1*x2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + 2*x1*x3*y2*y3 - 2*x1*x3*y2*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + 2*x1*x3*z2*z3 - 2*x1*x3*z2*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - 2*x1*x4*y2*y3 + 2*x1*x4*y2*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - 2*x1*x4*z2*z3 + 2*x1*x4*z2*z4 + x2^2*y3^2 - 2*x2^2*y3*y4 + x2^2*y4^2 + x2^2*z3^2 - 2*x2^2*z3*z4 + x2^2*z4^2 + 2*x2*x3*y1*y3 - 2*x2*x3*y1*y4 - 2*x2*x3*y2*y3 + 2*x2*x3*y2*y4 + 2*x2*x3*z1*z3 - 2*x2*x3*z1*z4 - 2*x2*x3*z2*z3 + 2*x2*x3*z2*z4 - 2*x2*x4*y1*y3 + 2*x2*x4*y1*y4 + 2*x2*x4*y2*y3 - 2*x2*x4*y2*y4 - 2*x2*x4*z1*z3 + 2*x2*x4*z1*z4 + 2*x2*x4*z2*z3 - 2*x2*x4*z2*z4 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - 2*x3*x4*y1^2 + 4*x3*x4*y1*y2 - 2*x3*x4*y2^2 - 2*x3*x4*z1^2 + 4*x3*x4*z1*z2 - 2*x3*x4*z2^2 + x4^2*y1^2 - 2*x4^2*y1*y2 + x4^2*y2^2 + x4^2*z1^2 - 2*x4^2*z1*z2 + x4^2*z2^2 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y2*z3^2 + 4*y1*y2*z3*z4 - 2*y1*y2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + 2*y1*y3*z2*z3 - 2*y1*y3*z2*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - 2*y1*y4*z2*z3 + 2*y1*y4*z2*z4 + y2^2*z3^2 - 2*y2^2*z3*z4 + y2^2*z4^2 + 2*y2*y3*z1*z3 - 2*y2*y3*z1*z4 - 2*y2*y3*z2*z3 + 2*y2*y3*z2*z4 - 2*y2*y4*z1*z3 + 2*y2*y4*z1*z4 + 2*y2*y4*z2*z3 - 2*y2*y4*z2*z4 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - 2*y3*y4*z1^2 + 4*y3*y4*z1*z2 - 2*y3*y4*z2^2 + y4^2*z1^2 - 2*y4^2*z1*z2 + y4^2*z2^2);
    t2 = (y4*x1^2*y2 - x1^2*y2*y3 + x1^2*y3^2 - y4*x1^2*y3 - x1^2*z2*z3 + z4*x1^2*z2 + x1^2*z3^2 - z4*x1^2*z3 + x1*x2*y1*y3 - y4*x1*x2*y1 + x1*x2*y2*y3 - y4*x1*x2*y2 - 2*x1*x2*y3^2 + 2*y4*x1*x2*y3 + x1*x2*z1*z3 - z4*x1*x2*z1 + x1*x2*z2*z3 - z4*x1*x2*z2 - 2*x1*x2*z3^2 + 2*z4*x1*x2*z3 + x1*x3*y1*y2 - 2*x1*x3*y1*y3 + y4*x1*x3*y1 - x1*x3*y2^2 + 2*x1*x3*y2*y3 - y4*x1*x3*y2 + x1*x3*z1*z2 - 2*x1*x3*z1*z3 + z4*x1*x3*z1 - x1*x3*z2^2 + 2*x1*x3*z2*z3 - z4*x1*x3*z2 - x4*x1*y1*y2 + x4*x1*y1*y3 + x4*x1*y2^2 - x4*x1*y2*y3 - x4*x1*z1*z2 + x4*x1*z1*z3 + x4*x1*z2^2 - x4*x1*z2*z3 - x2^2*y1*y3 + y4*x2^2*y1 + x2^2*y3^2 - y4*x2^2*y3 - x2^2*z1*z3 + z4*x2^2*z1 + x2^2*z3^2 - z4*x2^2*z3 - x2*x3*y1^2 + x2*x3*y1*y2 + 2*x2*x3*y1*y3 - y4*x2*x3*y1 - 2*x2*x3*y2*y3 + y4*x2*x3*y2 - x2*x3*z1^2 + x2*x3*z1*z2 + 2*x2*x3*z1*z3 - z4*x2*x3*z1 - 2*x2*x3*z2*z3 + z4*x2*x3*z2 + x4*x2*y1^2 - x4*x2*y1*y2 - x4*x2*y1*y3 + x4*x2*y2*y3 + x4*x2*z1^2 - x4*x2*z1*z2 - x4*x2*z1*z3 + x4*x2*z2*z3 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - x4*x3*y1^2 + 2*x4*x3*y1*y2 - x4*x3*y2^2 - x4*x3*z1^2 + 2*x4*x3*z1*z2 - x4*x3*z2^2 - y1^2*z2*z3 + z4*y1^2*z2 + y1^2*z3^2 - z4*y1^2*z3 + y1*y2*z1*z3 - z4*y1*y2*z1 + y1*y2*z2*z3 - z4*y1*y2*z2 - 2*y1*y2*z3^2 + 2*z4*y1*y2*z3 + y1*y3*z1*z2 - 2*y1*y3*z1*z3 + z4*y1*y3*z1 - y1*y3*z2^2 + 2*y1*y3*z2*z3 - z4*y1*y3*z2 - y4*y1*z1*z2 + y4*y1*z1*z3 + y4*y1*z2^2 - y4*y1*z2*z3 - y2^2*z1*z3 + z4*y2^2*z1 + y2^2*z3^2 - z4*y2^2*z3 - y2*y3*z1^2 + y2*y3*z1*z2 + 2*y2*y3*z1*z3 - z4*y2*y3*z1 - 2*y2*y3*z2*z3 + z4*y2*y3*z2 + y4*y2*z1^2 - y4*y2*z1*z2 - y4*y2*z1*z3 + y4*y2*z2*z3 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - y4*y3*z1^2 + 2*y4*y3*z1*z2 - y4*y3*z2^2)/(x1^2*y3^2 - 2*x1^2*y3*y4 + x1^2*y4^2 + x1^2*z3^2 - 2*x1^2*z3*z4 + x1^2*z4^2 - 2*x1*x2*y3^2 + 4*x1*x2*y3*y4 - 2*x1*x2*y4^2 - 2*x1*x2*z3^2 + 4*x1*x2*z3*z4 - 2*x1*x2*z4^2 - 2*x1*x3*y1*y3 + 2*x1*x3*y1*y4 + 2*x1*x3*y2*y3 - 2*x1*x3*y2*y4 - 2*x1*x3*z1*z3 + 2*x1*x3*z1*z4 + 2*x1*x3*z2*z3 - 2*x1*x3*z2*z4 + 2*x1*x4*y1*y3 - 2*x1*x4*y1*y4 - 2*x1*x4*y2*y3 + 2*x1*x4*y2*y4 + 2*x1*x4*z1*z3 - 2*x1*x4*z1*z4 - 2*x1*x4*z2*z3 + 2*x1*x4*z2*z4 + x2^2*y3^2 - 2*x2^2*y3*y4 + x2^2*y4^2 + x2^2*z3^2 - 2*x2^2*z3*z4 + x2^2*z4^2 + 2*x2*x3*y1*y3 - 2*x2*x3*y1*y4 - 2*x2*x3*y2*y3 + 2*x2*x3*y2*y4 + 2*x2*x3*z1*z3 - 2*x2*x3*z1*z4 - 2*x2*x3*z2*z3 + 2*x2*x3*z2*z4 - 2*x2*x4*y1*y3 + 2*x2*x4*y1*y4 + 2*x2*x4*y2*y3 - 2*x2*x4*y2*y4 - 2*x2*x4*z1*z3 + 2*x2*x4*z1*z4 + 2*x2*x4*z2*z3 - 2*x2*x4*z2*z4 + x3^2*y1^2 - 2*x3^2*y1*y2 + x3^2*y2^2 + x3^2*z1^2 - 2*x3^2*z1*z2 + x3^2*z2^2 - 2*x3*x4*y1^2 + 4*x3*x4*y1*y2 - 2*x3*x4*y2^2 - 2*x3*x4*z1^2 + 4*x3*x4*z1*z2 - 2*x3*x4*z2^2 + x4^2*y1^2 - 2*x4^2*y1*y2 + x4^2*y2^2 + x4^2*z1^2 - 2*x4^2*z1*z2 + x4^2*z2^2 + y1^2*z3^2 - 2*y1^2*z3*z4 + y1^2*z4^2 - 2*y1*y2*z3^2 + 4*y1*y2*z3*z4 - 2*y1*y2*z4^2 - 2*y1*y3*z1*z3 + 2*y1*y3*z1*z4 + 2*y1*y3*z2*z3 - 2*y1*y3*z2*z4 + 2*y1*y4*z1*z3 - 2*y1*y4*z1*z4 - 2*y1*y4*z2*z3 + 2*y1*y4*z2*z4 + y2^2*z3^2 - 2*y2^2*z3*z4 + y2^2*z4^2 + 2*y2*y3*z1*z3 - 2*y2*y3*z1*z4 - 2*y2*y3*z2*z3 + 2*y2*y3*z2*z4 - 2*y2*y4*z1*z3 + 2*y2*y4*z1*z4 + 2*y2*y4*z2*z3 - 2*y2*y4*z2*z4 + y3^2*z1^2 - 2*y3^2*z1*z2 + y3^2*z2^2 - 2*y3*y4*z1^2 + 4*y3*y4*z1*z2 - 2*y3*y4*z2^2 + y4^2*z1^2 - 2*y4^2*z1*z2 + y4^2*z2^2);
    dist2 = (x1 - x3 - t1*(x1 - x2) + t2*(x3 - x4))^2 + (y1 - y3 - t1*(y1 - y2) + t2*(y3 - y4))^2 + (z1 - z3 - t1*(z1 - z2) + t2*(z3 - z4))^2;
    
    if (t1<=1) && (t1>=0) && (t2<=1) && (t2>=0)
        dist_con(index) = dist2 - d_min^2;
    else
        dist_con(index) = inf;
    end
    
    index = index + 1;
    
    
    % %%%%% point to line collision check 1 %%%%%%
    % node number
    x1 = edge_set{k}(1,1);     y1 = edge_set{k}(1,2);     z1 = 0;
    x2 = edge_set{k}(3,1);     y2 = edge_set{k}(3,2);     z2 = 0;
    x3 = edge_set{k}(4,1);     y3 = edge_set{k}(4,2);     z3 = 0;

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        dist_con(index) = dist2 - d_min^2;
    else
        dist_con(index) = inf;
    end
    
    index = index + 1;
    
    
    % %%%%% point to line collision check 2 %%%%%%
    % node number
    x1 = edge_set{k}(2,1);     y1 = edge_set{k}(2,2);     z1 = 0;
    x2 = edge_set{k}(3,1);     y2 = edge_set{k}(3,2);     z2 = 0;
    x3 = edge_set{k}(4,1);     y3 = edge_set{k}(4,2);     z3 = 0;

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        dist_con(index) = dist2 - d_min^2;
    else
        dist_con(index) = inf;
    end
    
    index = index + 1;
    
    
    % %%%%% point to line collision check 3 %%%%%%
    % node number
    x1 = edge_set{k}(3,1);     y1 = edge_set{k}(3,2);     z1 = 0;
    x2 = edge_set{k}(1,1);     y2 = edge_set{k}(1,2);     z2 = 0;
    x3 = edge_set{k}(2,1);     y3 = edge_set{k}(2,2);     z3 = 0;

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        dist_con(index) = dist2 - d_min^2;
    else
        dist_con(index) = inf;
    end
    
    index = index + 1;
    

    % %%%%% point to line collision check 4 %%%%%%
    % node number
    x1 = edge_set{k}(4,1);     y1 = edge_set{k}(4,2);     z1 = 0;
    x2 = edge_set{k}(1,1);     y2 = edge_set{k}(1,2);     z2 = 0;
    x3 = edge_set{k}(2,1);     y3 = edge_set{k}(2,2);     z3 = 0;

    t0 = -(2*(x1 - x2)*(x2 - x3) + 2*(y1 - y2)*(y2 - y3) + 2*(z1 - z2)*(z2 - z3))/(2*(x2 - x3)^2 + 2*(y2 - y3)^2 + 2*(z2 - z3)^2);
    dist2 =  (x1 - x2 + t0*(x2 - x3))^2 + (y1 - y2 + t0*(y2 - y3))^2 + (z1 - z2 + t0*(z2 - z3))^2;

    if (t0<=1) && (t0>=0)
        dist_con(index) = dist2 - d_min^2;
    else
        dist_con(index) = inf;
    end
    
    index = index + 1;
    
end % for end
% 
% % plot
% figure
% hold on
% 
% for i = 1:length(obstacle)
%     obstacle_set = [obstacle{i}(1) obstacle{i}(3) ; 
%                     obstacle{i}(2) obstacle{i}(3) ; 
%                     obstacle{i}(2) obstacle{i}(4) ; 
%                     obstacle{i}(1) obstacle{i}(4) ; 
%                     obstacle{i}(1) obstacle{i}(3)];
%     plot(obstacle_set(:,1),obstacle_set(:,2),'k','linewidth',2)
%     patch(obstacle_set(:,1),obstacle_set(:,2),[0.6 0.6 0.6])
% end
% 
% plot(polygon([1 3 2],1), polygon([1 3 2],2))
function [n, param, Ground] = foot_initialize(n, L_nom)
% Generating structure "n", parameter "param", and Ground mesh "Ground"

%% Initial Position setting

environment_set = 10;

param.Dim = 3;
param.L_nom = L_nom;

param.maxIter = 10000;      % Maximum iteration for support polygon planning.
param.mixing_factor = 0;
graph_plot = 1;     % If set 1, draw foot planning.

n.pos = n.pos * L_nom;

switch environment_set
    case 0
        % Flat plane
        param.init_com = [0 0 0];
        param.goal = [3 0 0];
        param.stepsize = 0.15;
        limit = [-2 5 -2 5 -1 0];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;
        
        Ground_Point = [    -2, -2, 0;
                    -2, 5,  0;
                    5,  -2, 0;
                    5,  5,  0; ];

    case 1
        % Sloped plane one step 10 %
        L_nom = 1.25; % original trial: 1.25
        param.init_com = [0 0 0.2];
        param.goal = [1 0 0.2]; % original
        param.stepsize = 0.05;
        limit = [-2 5 -2 5 -1 0]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        Rot_Matrix = [1, 0, 0; 0, 10/(sqrt(101)), -1/(sqrt(101)); 0, 1/(sqrt(101)), 10/(sqrt(101));]; 
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*Rot_Matrix.'*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*Rot_Matrix.'*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*Rot_Matrix.'*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,3) = n.pos(4,3) + 0.2;
        n.pos(5,3) = n.pos(5,3) + 0.2;
        n.pos(6,3) = n.pos(6,3) + 0.2;
        
        Ground_Point = [    -2, -2, 0;
                            -2, 5,  0.7;
                            5,  -2, 0;
                            5,  5,  0.7; ];
        
    case 2
        % Sloped plane long locomotion 10%
        L_nom = 1.25; % original trial: 1.25
        param.init_com = [0 0 0.2];
        param.goal = [3 2 0.4]; % original
        param.stepsize = 0.05;
        limit = [-2 5 -2 5 -1 1]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        Rot_Matrix = [1, 0, 0; 0, 10/(sqrt(101)), -1/(sqrt(101)); 0, 1/(sqrt(101)), 10/(sqrt(101));]; 
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*Rot_Matrix.'*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*Rot_Matrix.'*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*Rot_Matrix.'*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,3) = n.pos(4,3) + 0.2;
        n.pos(5,3) = n.pos(5,3) + 0.2;
        n.pos(6,3) = n.pos(6,3) + 0.2;
        
        Ground_Point = [    -2, -2, 0;
                            -2, 5,  0.7;
                            5,  -2, 0;
                            5,  5,  0.7; ];
       
    case 3
        % Sloped plane long locomotion 20%
        L_nom = 1.25; % original trial: 1.25
        param.init_com = [0 0 0.4];
        param.goal = [3 3 1.0]; % original
        param.stepsize = 0.05;
        limit = [-2 5 -2 5 -1 1]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        Rot_Matrix = [1, 0, 0; 0, 10/(sqrt(104)), -2/(sqrt(104)); 0, 2/(sqrt(104)), 10/(sqrt(104));]; 
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*Rot_Matrix.'*L_nom + [0, 0, 0.1];
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*Rot_Matrix.'*L_nom + [0, 0, 0.1];
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*Rot_Matrix.'*L_nom + [0, 0, 0.1];
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,3) = n.pos(4,3) + 0.2;
        n.pos(5,3) = n.pos(5,3) + 0.2;
        n.pos(6,3) = n.pos(6,3) + 0.2;

        Ground_Point = [    -2, -2, 0;
                            -2, 5,  1.4;
                            5,  -2, 0;
                            5,  5,  1.4; ];       
        
    case 4
        % Valley locomotion
        L_nom = 1.25; % original trial: 1.25
        param.init_com = [0, 1.5+sqrt(3)/12*L_nom, sqrt(3)/4*L_nom];
        param.goal = [4, 1.5+sqrt(3)/12*L_nom, sqrt(3)/4*L_nom]; % original
        param.stepsize = 0.05;
        limit = [-2 6 -2 6 0 2]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        Rot_Matrix = [0, 1, 0; -1, 0, 0; 0, 0, 1;]; 
        Ground_Clearance = 0.11;
        n.pos(1,:) = n.pos(1,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        n.pos(2,:) = n.pos(2,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        n.pos(3,:) = n.pos(3,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        n.pos(4,:) = n.pos(4,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        n.pos(5,:) = n.pos(5,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        n.pos(6,:) = n.pos(6,:)*Rot_Matrix + param.init_com + [0, 0, Ground_Clearance];
        param.Start_T(1,:) = n.pos(1,:);
        param.Start_T(2,:) = n.pos(2,:);
        param.Start_T(3,:) = n.pos(3,:);
        
        Ground_Point = [    -2  -2,     3.5;
                            -2, 1.5,    0;
                            -2, 5,      3.5;
                            5,  -2,     3.5;
                            5,  1.5,    0;
                            5,  5,      3.5;    ];
        
    case 5
        % Testing member-ground collision algorithm (Thin wall at the middle)
        L_nom = 1.25;
        param.init_com = [0, 1.5, 0];
        param.goal = [3, 1.5, 0];
        param.stepsize = 0.05;
        limit = [-2 5 -2 5 0 1.5]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;
        
        Ground_Point = [    -2.     -2.     0;
                            -2,     5,      0;
                            5,      -2,     0;
                            5,      5,      0;
                            0.6,    -2,     0;
                            0.62,   -2,     0;
                            0.6,    5,      0;
                            0.62,   5,      0;
                            0.6,    1,      0;
                            0.62,   1,      0;
                            0.61,   0.9,    0;
                            0.6,    3,      0;
                            0.62,   3,      0;
                            0.61,   3.1,    0;
                            0.61,   1,      1.5;
                            0.61,   3,      1.5;    ];
        
    case 6
        % 45 deg incline plane locomotion
        L_nom = 1.25;
        param.init_com = [0, 0, 2];
        param.goal = [4, 4, 6];
        param.stepsize = 0.05;
        limit = [-2 5 -2 5 0 10]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        Rot_Matrix = [1, 0, 0; 0, sqrt(2)/2, -sqrt(2)/2; 0, sqrt(2)/2, sqrt(2)/2;]; 
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*Rot_Matrix.'*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*Rot_Matrix.'*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*Rot_Matrix.'*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,3) = n.pos(4,3) + 3.5;
        n.pos(5,3) = n.pos(5,3) + 3.5;
        n.pos(6,3) = n.pos(6,3) + 3.5;
        
        Ground_Point = [    -2, -2, 0;
                            5,  -2, 0;
                            -2, 5,  7;
                            5,  5,  7;  ];
        
    case 7
        L_nom = 1.25;
        param.init_com = [-1 -1 0];
        param.goal = [3 3 0];
        param.stepsize = 0.05;
        limit = [-2 6 -2 6 0 2.5];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom  + [0, 0, 0.1];
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom  + [0, 0, 0.1];
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom  + [0, 0, 0.1];
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;
        
        Ground_Point = [];
                        
        for HorGrid = -2 : 2 : 6
            for VerGrid = -2 : 2 : 6
                if (HorGrid <= 0) && (HorGrid >= -2) && (VerGrid <= 0) && (VerGrid >= -2)
                    Ground_Point = [Ground_Point; HorGrid, VerGrid, 0];
                else
                    Ground_Point = [Ground_Point; HorGrid, VerGrid, 1.5 * rand];
                end
            end
        end

    case 8
        % Flat plane
        param.init_com = [-4 -4 0];
        param.goal = [4 4 0];
        param.stepsize = 0.15;
        limit = [-3 9 -3 9 -1 1];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;
        
        Ground_Point = [    -3, -3, 0;
                    -3, 9,  0;
                    9,  -3, 0;
                    9,  9,  0; ];

        case 9
        % Flat plane
        param.init_com = [-6 -6 0];
        param.goal = [6 6 0];
        param.stepsize = 0.15;
        limit = [-10 10 -10 10 -0.1 0.1];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;
        
        Ground_Point = [    -10, -10, 0;
                    -10, 10,  0;
                    10,  -10, 0;
                    10,  10,  0; ];
    case 10
        % Testing member-ground collision algorithm (Thin wall at the middle)
        L_nom = 1.25;
        param.init_com = [-8, 8, 0];
        param.goal = [8, -8, 0];
        param.stepsize = 0.25;
        limit = [-10 15 -10 15 0 1.5]; % limit = [x_min x_max y_min y_max z_min z_max];
        param.Config_Limit = [limit(1) limit(2) ; limit(3) limit(4); limit(5) limit(6);];
        
        param.Start_T(1,:) = param.init_com + [-sqrt(3)/3 0 0]*L_nom;
        param.Start_T(2,:) = param.init_com + [sqrt(3)/6 0.5 0]*L_nom;
        param.Start_T(3,:) = param.init_com + [sqrt(3)/6 -0.5 0]*L_nom;
        n.pos(1,:) = param.Start_T(1,:);
        n.pos(2,:) = param.Start_T(2,:);
        n.pos(3,:) = param.Start_T(3,:);
        n.pos(4,:) = n.pos(4,:) + param.init_com;
        n.pos(5,:) = n.pos(5,:) + param.init_com;
        n.pos(6,:) = n.pos(6,:) + param.init_com;

        Ground_base = [     -12.     -12.     0;
                            -12,     12,      0;
                            12,      -12,     0;
                            12,      12,      0;];

%         Wall_1 = [          0.6,    -2,     0;
%                             0.62,   -2,     0;
%                             0.6,    5,      0;
%                             0.62,   5,      0;
%                             0.6,    1,      0;
%                             0.62,   1,      0;
%                             0.61,   0.9,    0;
%                             0.6,    3,      0;
%                             0.62,   3,      0;
%                             0.61,   3.1,    0;
%                             0.61,   1,      1.5;
%                             0.61,   3,      1.5;];

% Wall_1 = wall_create(Ground_base,[1 0.7],[3 0.7],5);
% Wall_2 = wall_create(Ground_base,[0.7 1],[0.7 3],5);
% Wall_3 = wall_create(Ground_base,[1 3.1],[3 3.1],5);

        Ground_Point = [    Ground_base];


%         Ground_Point = [    -2.     -2.     0;
%                             -2,     5,      0;
%                             5,      -2,     0;
%                             5,      5,      0;
%                             0.6,    -2,     0;
%                             0.62,   -2,     0;
%                             0.6,    5,      0;
%                             0.62,   5,      0;
%                             0.6,    1,      0;
%                             0.62,   1,      0;
%                             0.61,   0.9,    0;
%                             0.6,    3,      0;
%                             0.62,   3,      0;
%                             0.61,   3.1,    0;
%                             0.61,   1,      1.5;
%                             0.61,   3,      1.5;    ];

end



%% Initial Ground Setting

% param.MaxStaticCoeff = 0.5;


Ground = Ground_Generation(Ground_Point); 

% surface_normal_vector = [];
% 
% fixed_node = data.fixed_node{1};
% 
% for normal_index = 1 : length(fixed_node)
%     for ground_index = 1 : length(Ground)
%         temp_pos = n.pos(fixed_node(normal_index),:);
%         if inpolygon(n.pos(fixed_node(normal_index),1), n.pos(fixed_node(normal_index),2), Ground(ground_index).Vertices(1:3,1), Ground(ground_index).Vertices(1:3,2)) == 1
%             surface_normal_vector = [surface_normal_vector; NVector_Find(fixed_node(normal_index), Ground, ground_index)];
%             break;
%         end
%     end
% end

end

clear StartTree Final_Path_Start Final_Path_T obstacle

%% Setting one step trajectory

% Step toward node 2 - node 3 line

% Change the array shape to make calculation easier.

x_ini = data.x(1,:);

Node_Pos_ini = reshape(x_ini, [numel(x_ini)/3,3]);

% rotation line
Node_Pos_2 = Node_Pos_ini(2,:); % Node 2 position
Node_Pos_3 = Node_Pos_ini(3,:); % Node 3 position

Node_Pos_fin = [];              % Initialize final rotated Node positions

for k = 1 : length(Node_Pos_ini)
    Node_Pos_fin_Temp = Rotating_Line((Node_Pos_ini(k,:)).', Node_Pos_2.', Node_Pos_3.', -asin(2*sqrt(2)/3));
    Node_Pos_fin(k,:) = Node_Pos_fin_Temp.';
end

x_fin = reshape(Node_Pos_fin, [numel(Node_Pos_fin), 1]);

%% Enter x_fin into tree structure

Final_Path_T(1).n = Node_Pos_ini(1:3,:);
Final_Path_T(2).n = Node_Pos_fin(2:4,:);

%% Plot

figure()
for i = 1:2
    hold on
    plot3(Final_Path_T(i).n([1 2 3 1],1),Final_Path_T(i).n([1 2 3 1],2),Final_Path_T(i).n([1 2 3 1],3),'g','linewidth',2)
    axis equal
    axis([-2 2 -2 2 0 2]); 
    
    for j = 1:length(n.elist)

        a = n.elist(j,1);
        b = n.elist(j,2);

        Xa = Node_Pos_ini(a,1);
        Ya = Node_Pos_ini(a,2);
        Za = Node_Pos_ini(a,3);
        Xb = Node_Pos_ini(b,1);
        Yb = Node_Pos_ini(b,2);
        Zb = Node_Pos_ini(b,3);

        plot3([Xa Xb],[Ya Yb],[Za Zb],'b','LineWidth',2)

    end
    for j = 1:length(n.elist)

        a = n.elist(j,1);
        b = n.elist(j,2);

        Xa = Node_Pos_fin(a,1);
        Ya = Node_Pos_fin(a,2);
        Za = Node_Pos_fin(a,3);
        Xb = Node_Pos_fin(b,1);
        Yb = Node_Pos_fin(b,2);
        Zb = Node_Pos_fin(b,3);

        plot3([Xa Xb],[Ya Yb],[Za Zb],'r','LineWidth',2)

    end
end
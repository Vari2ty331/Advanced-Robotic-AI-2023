% Trajectory planning of center of mass based on planned support polygon

%% Method 1: Connecting center of mass
% com_step_num = 100; % The number of step between two center of mass
% 
% dx_cm_desire = [];
% foot_desire = [];
% for i = 1:length(Final_Path_T)-1
%     com1 = sum(Final_Path_T(i).n) / 3;
%     com2 = sum(Final_Path_T(i+1).n) / 3;
%     vec = (com2 - com1)/com_step_num;
%     dx_cm_desire = [dx_cm_desire ; vec(1)*ones(com_step_num,1) vec(2)*ones(com_step_num,1) zeros(com_step_num,1)];
%     foot_desire = [foot_desire ; Final_Path_T(i+1).n(3,:) 0];
% end

%% Method 2: Connecting center of mass and middle point of edge
step_num = 100;
scale_factor = 1.25;
scale = data.desired_vel_max/0.015*scale_factor; % original velocity max: 0.015;
com_step_size = sqrt(3)/3/step_num*scale;

dx_cm_desire = [];
foot_desire = [];

% vec_data = [];
for i = 1:length(Final_Path_T)-1
    com1 = sum(Final_Path_T(i).n) / 3;
    com2 = sum(Final_Path_T(i+1).n) / 3;
    
    com = com1;
    P_mid = sum(Final_Path_T(i+1).n(1:2,:)) / 2;
    com_step_num = round( norm(P_mid - com) / com_step_size );
    vec = (P_mid - com)/com_step_num;
    dx_cm_desire = [dx_cm_desire ; vec(1)*ones(com_step_num,1) vec(2)*ones(com_step_num,1) vec(3)*ones(com_step_num,1)];
%     vec_data = [vec_data norm(vec)];
    
    com = com2;
    P_mid = sum(Final_Path_T(i+1).n(1:2,:)) / 2;
    com_step_num = round( norm(P_mid - com) / com_step_size );
    vec = (com - P_mid)/com_step_num;
    dx_cm_desire = [dx_cm_desire ; vec(1)*ones(com_step_num,1) vec(2)*ones(com_step_num,1) vec(3)*ones(com_step_num,1)];
%     vec_data = [vec_data norm(vec)];
    
    foot_desire = [foot_desire ; Final_Path_T(i+1).n(3,:)];
end

%% temp plot
% figure
% desired_path = [0,0,0];
% 
% index = 1;
% for i = 1:length(dx_cm_desire)
%     desired_path(index+1,:) = desired_path(index,:) + dx_cm_desire(index,:)*dt;
%     index = index + 1;
% end
% 
% plot(desired_path(:,1), desired_path(:,2),'m.')
% hold on 
% 
% for i = 1:length(Final_Path_T)
%     plot(Final_Path_T(i).n([1 2 3 1],1),Final_Path_T(i).n([1 2 3 1],2),'g','linewidth',1)
% end
% 
% axis equal
% 
% % figure
% % plot(vec_data)
% % ylim([0 max(vec_data)]);

function [L, mass_sphere, mass_member, com_member] = com_member_cal(elist,pos)
% In this code, set mass of sphere, member and its center of mass

L = lengths(elist,pos);

mass_set = 1;
% 1: Actual value
% 2: 3 Member test bench value
% 3: Assumes weight concenterated in node
% 4: Some members replaced by aluminum profile

switch mass_set
    case 1
       %% Actual value
        mass_sphere = 0.37*9.8*ones(length(pos),1); % sphere: 0.37 kg (by Alex. Dec. 2)

        for i = 1:length(elist)
            % 191227 Actually measured center of mass
            mass_member{i} = [0.96 0.23 2.00]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + member-end + battery
            com_member{i} = [0.3135 L(i)/2 L(i)-0.4375]; % position of center of mass 

%             % Assume symmetric
%             mass_member{i} = [2.00 0.23 2.00]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + member-end + battery
%             com_member{i} = [0.4375 L(i)/2 L(i)-0.4375]; % position of center of mass 
            
        end
        
    case 2
       %% 3 Member Test bench value (Alex's Dimension)
        % mass_sphere = zeros(length(pos),1);
        mass_sphere(4) = (0.37 + 0)*9.8; % sphere: 0.37 kg (by Alex. Dec. 2)

        % there's no edge module for edge 1,2,3. But gives litte mass to avoid singularity during calculation
        for i = 1:3
            mass_member{i} = [1e-3 1e-3 1e-3]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + member-end + battery
            com_member{i} = [0.308 L(i)/2 L(i)-0.425]; % position of center of mass
        end

        % bottom member (neglect gimbal mass)
        mass_member{4} = [0.380 0.260 2.000+5.56]*9.8; % mass of gimbal + tensioner + battery,   band,   friction drive + member-end + battery
        com_member{4} = [0.150 L(4)/2 L(4)-0.425+0.225]; % position of center of mass

        % upper members (neglect gimbal mass)
        for i = 5:6
            mass_member{i} = [0.96 0.260 1.420]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + gimbal + battery
            com_member{i} = [0.308 L(i)/2 L(i)-0.230]; % position of center of mass     
        end
        
    case 3
       %% Assumes weight is concenterated in node
        % Upenn assumption (2019.11.01.): mass of node 100 N 
        % Actual mass of node of 3 member: 40 N (2 tensioner + 1 drive + 3 member-end)
        mass_sphere = 40*ones(length(pos),1); 
        
        for i = 1:length(elist)
            % dummy mass
            mass_member{i} = [1e-3 1e-3 1e-3]*9.8;
            com_member{i} = [0 0 0];
        end
        
    case 4
       %% Some members replaced by aluminum profile
        mass_sphere = 0.37*9.8*ones(length(pos),1); % sphere: 0.37 kg (by Alex. Dec. 2)

        for i = 1:length(elist)
            % 191227 Actually measured center of mass
            mass_member{i} = [0.96 0.23 2.00]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + member-end + battery
            com_member{i} = [0.3135 L(i)/2 L(i)-0.4375]; % position of center of mass  
        end
        
        AL_index = [1 9 12];
        for i = AL_index
            mass_member{i} = [0.58 0.5 0.58]*9.8; % mass of tensioner + member-end + bettery,   band,   friction drive + member-end + battery
            com_member{i} = [0.2301 L(i)/2 L(i)-0.2301]; % position of center of mass
        end
        
end



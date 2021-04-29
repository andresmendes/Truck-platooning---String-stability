%% Truck platooning - String stability
% Animation of a string stable four-truck platooning at 72 km/h with 1
% second time gap.
%
%%

clear ; close all ; clc

%% Scenario

N_trucks = 4;                   % Number of trucks

% Parameters
tf      = 40;                   % Final time                            [s]
fR      = 30;                   % Frame rate                            [fps]
dt      = 1/fR;                 % Time resolution                       [s]
time    = linspace(0,tf,tf*fR); % Time                                  [s]

% Road
distance_analysis   = 150;      % Distance of analysis                  [m]
trackWidth          = 20;       % Track width                           [m]
laneMargin          = 2;        % Margin lane                           [m]

% Truck 1 (Leading truck)
truck_1_length      = 20;       % Length of the leading truck           [m]
truck_1_width       = 4;        % Width of the leading truck            [m]
truck_1_initial_speed    = 72/3.6;   % Speed of the leading truck       [m/s]
truck_1_initial_position = 150; % Initial position of the leading truck [m]

% Truck 2 (Second truck)
truck_2_length      = 20;       % Length of the second truck            [m]
truck_2_width       = 4;        % Width of the second truck             [m]
truck_2_initial_speed    = 72/3.6;   % Speed of the second truck        [m/s]
truck_2_initial_position = 120;  % Initial position of the second truck  [m]

% Truck 3 (Third truck)
truck_3_length      = 20;       % Length of the third truck            [m]
truck_3_width       = 4;        % Width of the third truck             [m]
truck_3_initial_speed    = 72/3.6;   % Speed of the third truck        [m/s]
truck_3_initial_position = 80;  % Initial position of the third truck  [m]

% Truck 4 (Fourth truck)
truck_4_length      = 20;       % Length of the third truck            [m]
truck_4_width       = 4;        % Width of the third truck             [m]
truck_4_initial_speed    = 72/3.6;   % Speed of the third truck        [m/s]
truck_4_initial_position = 40;  % Initial position of the third truck  [m]

% PARAMETERS 
% Struct to ode45
parameters.truck_1_length = truck_1_length;
parameters.truck_2_length = truck_2_length;
parameters.truck_3_length = truck_3_length;
parameters.truck_4_length = truck_4_length;

%% Simulation

% TIME
TSPAN = time;                   % Simulation time span  [s]

% INITIAL CONDITIONS
% Pair position and speed of first truck, pair position and speed of the
% second truck and so on ...
states_initial_condition = [truck_1_initial_position 
                            truck_1_initial_speed
                            truck_2_initial_position
                            truck_2_initial_speed
                            truck_3_initial_position
                            truck_3_initial_speed
                            truck_4_initial_position
                            truck_4_initial_speed];

% SIMULATION
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[TOUT,YOUT] = ode45(@(t,z) simulation(t,z,parameters),TSPAN,states_initial_condition,options);

% RETRIEVING STATES
% Truck 1
truck_1_position    = YOUT(:,1);
truck_1_speed       = YOUT(:,2);
% Truck 2
truck_2_position    = YOUT(:,3);
truck_2_speed       = YOUT(:,4);
% Truck 3
truck_3_position    = YOUT(:,5);
truck_3_speed       = YOUT(:,6);
% Truck 4
truck_4_position    = YOUT(:,7);
truck_4_speed       = YOUT(:,8);

% Acceleration
% Preallocating
truck_1_acc         = zeros(1,length(TOUT));
truck_2_acc         = zeros(1,length(TOUT));
truck_3_acc         = zeros(1,length(TOUT));
truck_4_acc         = zeros(1,length(TOUT));
for i=1:length(TOUT)
    [dz]    = simulation(TOUT(i),YOUT(i,:),parameters);
    truck_1_acc(i)  = dz(2);
    truck_2_acc(i)  = dz(4);
    truck_3_acc(i)  = dz(6);
    truck_4_acc(i)  = dz(8);
end

% Distances
dist_1_2 = truck_1_position - truck_2_position - truck_1_length;
dist_2_3 = truck_2_position - truck_3_position - truck_2_length;
dist_3_4 = truck_3_position - truck_4_position - truck_3_length;

%% Results

c = cool(N_trucks); % Colormap
    
figure
set(gcf,'Position',[270 140 1280 720])

% Create and open video writer object
v = VideoWriter('truck_platoon_string.avi');
v.Quality = 100;
open(v);

for i=1:length(time)
    subplot(3,2,1)
        hold on ; grid on
        position_max = max(truck_1_position);
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*position_max])
        cla 
        plot(TOUT,truck_1_position,'color',c(1,:))
        plot(TOUT,truck_2_position,'color',c(2,:))
        plot(TOUT,truck_3_position,'color',c(3,:))
        plot(TOUT,truck_4_position,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*position_max],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,3)
        hold on ; grid on
        speed_max = max(max([truck_1_speed truck_2_speed truck_3_speed truck_4_speed]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*speed_max])
        cla 
        plot(TOUT,truck_1_speed,'color',c(1,:))
        plot(TOUT,truck_2_speed,'color',c(2,:))
        plot(TOUT,truck_3_speed,'color',c(3,:))
        plot(TOUT,truck_4_speed,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*speed_max],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,2)
        hold on ; grid on
        acc_min = min(min([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        acc_max = max(max([truck_1_acc truck_2_acc truck_3_acc truck_4_acc]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[1.2*acc_min 1.2*acc_max])
        cla 
        plot(TOUT,truck_1_acc,'color',c(1,:))
        plot(TOUT,truck_2_acc,'color',c(2,:))
        plot(TOUT,truck_3_acc,'color',c(3,:))
        plot(TOUT,truck_4_acc,'color',c(4,:))
        plot([time(i) time(i)],[1.2*acc_min 1.2*acc_max],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
        legend('Truck 1','Truck 2','Truck 3','Truck 4','location','SouthEast')
    subplot(3,2,4)
        hold on ; grid on
        dist_max = max(max([dist_1_2 dist_2_3 dist_3_4]));
        set(gca,'xlim',[0 TOUT(end)],'ylim',[0 1.2*dist_max])
        cla 
        plot(TOUT,dist_1_2,'color',c(2,:))
        plot(TOUT,dist_2_3,'color',c(3,:))
        plot(TOUT,dist_3_4,'color',c(4,:))
        plot([time(i) time(i)],[0 1.2*dist_max],'k--') 
        xlabel('Time [s]')
        ylabel('Distance [m]')
        title('Separation Distance')
        legend('Trucks 1 & 2','Trucks 2 & 3','Trucks 3 & 4','location','SouthEast')
    subplot(3,2,5:6)
        hold on ; axis equal
        cla 
        % Position of the leading truck at instant [m]
        truck_1_position_inst = truck_1_position(i);
        truck_2_position_inst = truck_2_position(i);
        truck_3_position_inst = truck_3_position(i);
        truck_4_position_inst = truck_4_position(i);

        sideMarkingsX = [truck_1_position_inst-distance_analysis truck_1_position_inst];
        set(gca,'xlim',[truck_1_position_inst-distance_analysis truck_1_position_inst],'ylim',[-trackWidth/2-laneMargin +trackWidth/2+laneMargin])

        plot(sideMarkingsX,[+trackWidth/2 +trackWidth/2],'k--') % Left marking
        plot(sideMarkingsX,[-trackWidth/2 -trackWidth/2],'k--') % Right marking

        % DIMENSIONS
        % Truck 1
        truck_1_dimension_X = [truck_1_position_inst truck_1_position_inst truck_1_position_inst-truck_1_length truck_1_position_inst-truck_1_length];
        truck_1_dimension_Y = [+truck_1_width/2 -truck_1_width/2 -truck_1_width/2 +truck_1_width/2];
        % Truck 2
        truck_2_dimension_X = [truck_2_position_inst truck_2_position_inst truck_2_position_inst-truck_2_length truck_2_position_inst-truck_2_length];
        truck_2_dimension_Y = [+truck_2_width/2 -truck_2_width/2 -truck_2_width/2 +truck_2_width/2];
        % Truck 3
        truck_3_dimension_X = [truck_3_position_inst truck_3_position_inst truck_3_position_inst-truck_3_length truck_3_position_inst-truck_3_length];
        truck_3_dimension_Y = [+truck_3_width/2 -truck_3_width/2 -truck_3_width/2 +truck_3_width/2];
        % Truck 4
        truck_4_dimension_X = [truck_4_position_inst truck_4_position_inst truck_4_position_inst-truck_4_length truck_4_position_inst-truck_4_length];
        truck_4_dimension_Y = [+truck_4_width/2 -truck_4_width/2 -truck_4_width/2 +truck_4_width/2];
        
        % Plotting trucks
        fill(truck_1_dimension_X,truck_1_dimension_Y,c(1,:))
        fill(truck_2_dimension_X,truck_2_dimension_Y,c(2,:))
        fill(truck_3_dimension_X,truck_3_dimension_Y,c(3,:))
        fill(truck_4_dimension_X,truck_4_dimension_Y,c(4,:))
    
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
        
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end

close(v);

%% Auxiliary functions

function dz = simulation(t,z,parameters)
    % PARAMETERS
    truck_1_length = parameters.truck_1_length;
    truck_2_length = parameters.truck_2_length;
    truck_3_length = parameters.truck_3_length;

    % RETRIEVING STATES
    % Truck 1
    truck_1_position    = z(1);
    truck_1_speed       = z(2);
    truck_1_states      = [truck_1_position truck_1_speed];
    % Truck 2
    truck_2_position    = z(3);
    truck_2_speed       = z(4);
    truck_2_states      = [truck_2_position truck_2_speed];
    % Truck 3
    truck_3_position    = z(5);
    truck_3_speed       = z(6);
    truck_3_states      = [truck_3_position truck_3_speed];
    % Truck 4
    truck_4_position    = z(7);
    truck_4_speed       = z(8);
    truck_4_states      = [truck_4_position truck_4_speed];
    
    % SENSORS
    % Truck 2
    Truck_2_sensors.distance_preceding = (truck_1_position-truck_1_length) - truck_2_position;
    Truck_2_sensors.speed_preceding = truck_1_speed; 
    % Truck 3
    Truck_3_sensors.distance_preceding = (truck_2_position-truck_2_length) - truck_3_position;
    Truck_3_sensors.speed_preceding = truck_2_speed; 
    % Truck 4
    Truck_4_sensors.distance_preceding = (truck_3_position-truck_3_length) - truck_4_position;
    Truck_4_sensors.speed_preceding = truck_3_speed; 
    
    % DYNAMIC MODELS
    % Truck 1
    truck_1_derivative_states = truck_model(t,truck_1_states,1,1);
    truck_2_derivative_states = truck_model(t,truck_2_states,2,Truck_2_sensors);
    truck_3_derivative_states = truck_model(t,truck_3_states,3,Truck_3_sensors);
    truck_4_derivative_states = truck_model(t,truck_4_states,4,Truck_4_sensors);
    
    % OUTPUT STATES
    % Truck 1
    dz(1,1) = truck_1_derivative_states(1,1);
    dz(2,1) = truck_1_derivative_states(2,1);
    % Truck 2
    dz(3,1) = truck_2_derivative_states(1,1);
    dz(4,1) = truck_2_derivative_states(2,1);
    % Truck 3
    dz(5,1) = truck_3_derivative_states(1,1);
    dz(6,1) = truck_3_derivative_states(2,1);
    % Truck 4
    dz(7,1) = truck_4_derivative_states(1,1);
    dz(8,1) = truck_4_derivative_states(2,1);

end

function dstates = truck_model(~,states,truck_flag,truck_sensors)
    % truck_flag indicates the current truck

    % Parameters
    m   = 40000;                % Mass                      [kg]
    g   = 9.81;                 % Gravity                   [m/s2]
    Cd  = 0.78;                 % Drag coefficient          [-]
    A   = 10;                   % Frontal area              [m2]
    rho = 1;                    % Air density               [kg/m2]

    % States
%     X = states(1);
    V = states(2);

    % Drag resistance
    C  = 0.5*rho*Cd*A;
    Dx = C*V^2;

    % Rolling resistance
    Rx=0;
    
    % Gravity force
    theta   = 0;                % Road slope                [rad]
    Gx      = m*g*sin(theta);   %                           [N]

    if truck_flag == 1
        % CC
        V_r = 20;               % Reference speed           [m/s]
        Kp  = 500;              % Controller gain
        Ft  = Kp*(V_r - V) + Dx; % Longitudinal force       [N]
    else
        sensor_distance_preceding = truck_sensors.distance_preceding;
        sensor_speed_preceding = truck_sensors.speed_preceding;
        % ACC
        th      = 1.0;          % Time gap                  [s]
        desired_distance = th*V + 0;
        Kp      = 10000;
        Kd      = 10000;
        Ft = Kp*(sensor_distance_preceding - desired_distance) + Kd*(sensor_speed_preceding - V) + Dx;    
    end

    % Vehicle Dynamics
    dstates(1,1) = V;
    dstates(2,1) = (Ft - Dx - Rx - Gx)/m;
    
end

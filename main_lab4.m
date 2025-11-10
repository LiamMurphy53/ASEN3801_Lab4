clc;
clear;
close all;

%%section activation
%part 1:
run_141 = true;
run_142 = false;

%% Part 1
data=load('RSdata_nocontrol.mat');

time = data.rt_estim.time(:);

aircraft_state_array = data.rt_estim.signals.values';

motorForces = data.rt_motor.signals.values;
km = 0.0024;
d = 0.06;
inertMatrix = [-1, -1, -1, -1;
    -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2);
    d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2);
    km, -km, km, -km];

control_input_array = inertMatrix * motorForces';

fig = [1, 2, 3, 4, 5, 6];

col = '-b';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col);


const = struct();
const.g = 9.81; %m/s^2
const.m = 0.068; %kg
const.km = 0.0024; %N*m/(N)
const.d = 0.06; %m
const.nu = 1*10^(-3); %N/(m/s)^2
const.mu = 2*10^(-6); %N*m/(rad/s)^2
const.I = [5.8*10^(-5), 0, 0; 0, 7.2*10^(-5), 0; 0, 0, 1*10^(-4)]; %kg*m^2
tspan = [0, 10]; %seconds
            % x y z, phi, theta, psi, u, v, w p, q, r
%statevector_0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
%statevector_0 = [10, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% motor forces calculation at trim
%f = (const.m*const.g)/4;
%motor_forces = [f, f, f, f];

% 1.4.1: yaw=0
if(run_141)
    psi = 0; % deg
    psi = psi * pi/180;
    
    theta = atan((-25 *const.nu)/(const.m*const.g)); % rad
    
    f = (const.m * const.g * cos(theta) - 25 * const.nu * sin(theta))/ 4; % N
    motor_forces = [f, f, f, f];
    statevector_0 = [0, 0, 0, 0, theta, psi, 5*cos(theta), 0, 5*sin(theta), 0, 0, 0];


    [~, statevector] = ode45(@(t,statevector) QuadrotorEOM(t,statevector, const, motor_forces),tspan,statevector_0);


    figure()
    plot3(statevector(:,1),statevector(:,2),statevector(:,3),'LineWidth',2);

    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title("3D Position Trajectory" + " psi: 0 deg" + " theta: " + string(theta) + " rad" + " motor force: " + string(f) + " N");
    axis equal;
end

%1.4.2: yaw = 90
if(run_142)
    psi = 90; % deg
    psi = psi * (pi/180);
    
    phi = atan((-25 *const.nu)/(const.m*const.g)); % rad
    
    f = (const.m * const.g * cos(phi) - 25 * const.nu * sin(phi))/ 4; % N
    motor_forces = [f, f, f, f];
    statevector_0 = [0, 0, 0, phi, 0, psi, 0, -5*cos(phi), 5*sin(phi), 0, 0, 0];
    


    [~, statevector] = ode45(@(t,statevector) QuadrotorEOM(t,statevector, const, motor_forces),tspan,statevector_0);


    figure()
    plot3(statevector(:,1),statevector(:,2),statevector(:,3),'LineWidth',2);
    
    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title("3D Position Trajectory" + " psi: 90 deg" + " phi: " + string(phi) + " rad" + " motor force: " + string(f) + " N");
    axis equal;
end

[t, statevector] = ode45(@(t,statevector) QuadrotorEOM(t,statevector, const, motor_forces),tspan,statevector_0);





%% Functions

function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig, col)
    figure(fig(1));
    subplot(3,1,1);
    plot(time, aircraft_state_array(1,:), col); hold on;
    xlabel('Time (s)');
    ylabel('Meters');
    title('Inertial X Position');
    subplot(3,1,2);
    plot(time, aircraft_state_array(2,:), col); hold on;
    xlabel('Time (s)');
    ylabel('Meters');
    title('Inertial Y Position');
    subplot(3,1,3);
    plot(time, aircraft_state_array(3,:), col); hold on;
    xlabel('Time (s)');
    ylabel('Meters');
    title('Inertial Z Position');

    figure(fig(2));
    subplot(3,1,1);
    plot(time, aircraft_state_array(4, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Radians');
    title('Euler Phi Angle');
    subplot(3,1,2);
    plot(time, aircraft_state_array(5, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Radians');
    title('Euler Theta Angle');
    subplot(3,1,3);
    plot(time, aircraft_state_array(6, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Radians');
    title('Euler Psi Angle');

    figure(fig(3));
    subplot(3,1,1);
    plot(time, aircraft_state_array(7, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('X Velocity');
    subplot(3,1,2);
    plot(time, aircraft_state_array(8, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Y Velocity');
    subplot(3,1,3);
    plot(time, aircraft_state_array(9, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Z Velocity');

    figure(fig(4));
    subplot(3,1,1);
    plot(time, aircraft_state_array(10, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Rad/s');
    title('Roll Rate');
    subplot(3,1,2);
    plot(time, aircraft_state_array(11, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Rad/s');
    title('Pitch Rate');
    subplot(3,1,3);
    plot(time, aircraft_state_array(12, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Rad/s');
    title('Yaw Rate');

    figure(fig(5));
    subplot(4,1,1);
    plot(time, control_input_array(1, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Newtons');
    title('Vertical Thrust');
    subplot(4,1,2);
    plot(time, control_input_array(2, :), col); hold on;
    xlabel('Time (s)');
    ylabel('N*m');
    title('Roll Moment');
    subplot(4,1,3);
    plot(time, control_input_array(3, :), col); hold on;
    xlabel('Time (s)');
    ylabel('N*m');
    title('Pitch Moment');
    subplot(4,1,4);
    plot(time, control_input_array(4, :), col); hold on;
    xlabel('Time (s)');
    ylabel('N*m');
    title('Yaw Moment');

    figure(fig(6));
    x = aircraft_state_array(1, :);
    y = aircraft_state_array(2, :);
    z = aircraft_state_array(3, :);

    t = linspace(0, 1, numel(x));   

    scatter3(x, y, z, 5, t, 'filled'); 
    colormap([linspace(0,1,256)', linspace(1,0,256)', zeros(256,1)]); 

    grid on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('3D Position Trajectory');
    axis equal;
    view(3);
end

function var_dot = QuadrotorEOM(t, var, const, motor_forces)
    %decompose
    x = var(1);
    y = var(2);
    z = var(3);
    u = var(7);
    v = var(8);
    w = var(9);
    phi = var(4);
    theta = var(5);
    psi = var(6);
    p = var(10);
    q = var(11);
    r = var(12);

    v_e = [u, v, w];
    o = [p, q, r];
    


    %kinematics

    pos_dot = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); 
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
        -1*sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)] * v_e';
    e_dot = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi), -sin(phi);
        0, sin(phi)*sec(theta), cos(phi)*sec(theta)] * o';

    %dynamics

    %control forces and moments
    A = [ -1, -1, -1, -1;
    -const.d/sqrt(2), -const.d/sqrt(2), const.d/sqrt(2), const.d/sqrt(2);
    const.d/sqrt(2), -const.d/sqrt(2), -const.d/sqrt(2), const.d/sqrt(2);
    const.km, -const.km, const.km, -const.km];

    control = A*motor_forces';
    Zc = control(1);
    Lc = control(2);
    Mc = control(3);
    Nc = control(4);

    %gravity
    W = const.m*const.g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];

    % Aero forces and moments
     FdB = -const.nu*norm(v_e)*v_e';        
     MdB = -const.mu*norm(o)*o';

    v_e_dot = [r*v-q*w; p*w-r*u; q*u-p*v] + (1/const.m)*(W + [0; 0; Zc] + FdB);
    
    Ix = const.I(1,1);
    Iy = const.I(2,2);
    Iz = const.I(3,3);

    omega = [((Iy-Iz)/Ix)*q*r; ((Iz-Ix)/Iy)*p*r; ((Ix-Iy)/Iz)*p*q] + [1/Ix, 0, 0; 0, 1/Iy, 0; 0, 0, 1/Iz]*([Lc; Mc; Nc] + MdB);

    var_dot = [pos_dot(1); pos_dot(2); pos_dot(3); v_e_dot(1); v_e_dot(2); v_e_dot(3); e_dot(1); e_dot(2); e_dot(3); omega(1); omega(2); omega(3)];
end

function [Fc, Gc] = InnerLoopFeedback(var)
    k_spin = 0;

end
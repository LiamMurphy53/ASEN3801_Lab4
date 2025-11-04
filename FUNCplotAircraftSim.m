clc
clear
close

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
    title('Euler Psi Angle');
    subplot(3,1,2);
    plot(time, aircraft_state_array(5, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Radians');
    title('Euler Theta Angle');
    subplot(3,1,3);
    plot(time, aircraft_state_array(6, :), col); hold on;
    xlabel('Time (s)');
    ylabel('Radians');
    title('Euler Phi Angle');

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
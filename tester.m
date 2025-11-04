clc
clear
close

g   = 9.81;
m   = 0.068;
Ixx = 5.8e-5;
Iyy = 7.2e-5;
Izz = 1.0e-4;
I   = diag([Ixx Iyy Izz]);

deltaFc = [0; 0; 0];        
deltaGc = [0; 0; 0];

var0 = zeros(12,1);

tspan = [0 10];

[t_lin, var_lin] = ode45(@(t,var) linearizedQuadrotorEOM(t, var, g, m, I, deltaFc, deltaGc),tspan, var0);

trim_state = zeros(12,1);                  % xE=yE=zE=phi=theta=psi=0, etc.
aircraft_state_array_lin = (trim_state + var_lin.').';

npts = length(t_lin);
control_input_array_lin = zeros(4, npts);

fig = 1:6;
col = '-r';

FUNCplotAircraftSim(t_lin, aircraft_state_array_lin.', control_input_array_lin, fig, col);
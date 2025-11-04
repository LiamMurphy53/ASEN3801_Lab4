clc
clear
close

g   = 9.81;
m   = 0.068;
Ixx = 5.8e-5;
Iyy = 7.2e-5;
Izz = 1.0e-4;
I   = diag([Ixx Iyy Izz]);
d   = 0.06;           
km  = 0.0024;
nu  = 0;             
mu  = 0;

motorForces = (m*g/4) * ones(4,1);

deltaFc = [0; 0; 0];        
deltaGc = [0; 0; 0];

var0 = zeros(12,1);
var0(4) = deg2rad(5);
var0(5) = deg2rad(10);

tspan = [0 10];

[t_lin, var_lin] = ode45(@(t,var) linearizedQuadrotorEOM(t, var, g, m, I, deltaFc, deltaGc),tspan, var0);

trim_state = zeros(12,1);                
aircraft_state_array_lin = (trim_state + var_lin.').';

numPoints = length(t_lin);
control_input_array_lin = zeros(4, numPoints);

fig = 1:6;
col = '-r';

FUNCplotAircraftSim(t_lin, aircraft_state_array_lin.', control_input_array_lin, fig, col);
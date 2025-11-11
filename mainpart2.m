
clear; close all; clc;

%% part 1

g  = 9.81;                 
m  = 0.068;               
d  = 0.060;               
km = 0.0024;              
I  = diag([5.8e-5, 7.2e-5, 1.0e-4]);  
nu = 1e-3;                 
mu = 2e-6;                 


motor = (m*g/4) * ones(4,1);     


tspan = [0 10];
x0 = zeros(12,1);


deg = @(x) x*pi/180;
initalCons = {
    x0 + [0 0 0  deg(5) 0      0   0 0 0  0   0   0 ]';   % +roll
    x0 + [0 0 0  0      deg(5) 0   0 0 0  0   0   0 ]';   % +pitch
    x0 + [0 0 0  0      0      deg(5) 0 0 0  0   0   0 ]';% +yaw
    x0 + [0 0 0  0      0      0   0 0 0  0.1 0   0 ]';   % +p
    x0 + [0 0 0  0      0      0   0 0 0  0   0.1 0 ]';   % +q
    x0 + [0 0 0  0      0      0   0 0 0  0   0   0.1]'}; % +r

cols = {'-g','-r','-b','-m','-c','-k'};

f = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, nu, mu, motor);

figs = [1 2 3 4 5 6];  

for k = 1:numel(initalCons)
    x_ic = initalCons{k};
    [t, x] = ode45(f, tspan, x_ic);

    Zc = -sum(motor);  control = [Zc; 0; 0; 0] + zeros(4, numel(t));

    figs_case = figs + 6*(k-1);
    PlotAircraftSim(t.', x.', control, figs_case, cols{k});
end



%% part 2
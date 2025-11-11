
clear; close all; clc;

%% part 1

clear;close all ;clc;

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

clear; clc; close all;

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
initalCons = { ...
    x0 + [0 0 0  deg(5) 0      0     0 0 0   0   0   0 ]';   % +roll
    x0 + [0 0 0  0      deg(5) 0     0 0 0   0   0   0 ]';   % +pitch
    x0 + [0 0 0  0      0      deg(5) 0 0 0  0   0   0 ]';   % +yaw
    x0 + [0 0 0  0      0      0     0 0 0   0.1 0   0 ]';   % +p
    x0 + [0 0 0  0      0      0     0 0 0   0   0.1 0 ]';   % +q
    x0 + [0 0 0  0      0      0     0 0 0   0   0   0.1]'}; % +r

figs0 = [1 2 3 4 5 6];

%nonLinear
fNL  = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, nu, mu, motor);

% linear
deltaFc = zeros(3,1); deltaGc = zeros(3,1);
if exist('linearizedQuadrotorEOM','file')
    fLIN = @(t,dx) linearizedQuadrotorEOM(t, dx, g, m, I, deltaFc, deltaGc);
else
    fLIN = @(t,dx) QuadrotorEOM_Linearized(t, dx, g, m, I, deltaFc, deltaGc);
end

Zc = -sum(motor);

for k = 1:numel(initalCons)
    x_ic  = initalCons{k};    
    dx_ic = x_ic - x0;         

    [tNL,  xNL]   = ode45(fNL,  tspan, x_ic);
    [tLIN, dxLIN] = ode45(fLIN, tspan, dx_ic);
    xLIN = dxLIN + x0.';       

    
    uNL  = repmat([Zc;0;0;0], 1, numel(tNL));
    uLIN = repmat([Zc;0;0;0], 1, numel(tLIN));

    
    figs_case = figs0 + 6*(k-1);

    
    PlotAircraftSim(tNL.',  xNL.',  uNL,  figs_case, 'r-');
    PlotAircraftSim(tLIN.', xLIN.', uLIN, figs_case, 'b--');

    
    for fid = figs_case
        fh  = figure(fid);
        axs = findall(fh,'Type','axes');
        for ax = axs.'
            set(findobj(ax,'-property','HandleVisibility'), 'HandleVisibility','off');
            hold(ax,'on');
            plot(ax, NaN, NaN, 'r-',  'LineWidth',1.4, 'DisplayName','Nonlinear',  'HandleVisibility','on');
            plot(ax, NaN, NaN, 'b--', 'LineWidth',1.4, 'DisplayName','Linearized','HandleVisibility','on');
            legend(ax,'Location','best'); legend(ax,'boxoff');
        end
    end

    fh3 = figure(figs_case(6));
    ax3 = findall(fh3,'Type','axes');
    if ~isempty(ax3)
        ax3 = ax3(1);
        hold(ax3,'on');
        plot3(ax3, xNL(:,1),  xNL(:,2),  xNL(:,3),  'r-',  'LineWidth',1.2); 
        plot3(ax3, xLIN(:,1), xLIN(:,2), xLIN(:,3), 'b--', 'LineWidth',1.2); 
    end
end

clear; close all; clc;


g  = 9.81;                 
m  = 0.068;               
d  = 0.060;               
km = 0.0024;              
I  = diag([5.8e-5, 7.2e-5, 1.0e-4]);  
nu = 1e-3;                 
mu = 2e-6;                 


f_hover = (m*g/4) * ones(4,1);     


tspan = [0 10];
x0 = zeros(12,1);


deg = @(x) x*pi/180;
ICs = { ...
    x0 + [0 0 0  deg(5) 0      0   0 0 0  0   0   0 ]';   % +roll
    x0 + [0 0 0  0      deg(5) 0   0 0 0  0   0   0 ]';   % +pitch
    x0 + [0 0 0  0      0      deg(5) 0 0 0  0   0   0 ]';% +yaw
    x0 + [0 0 0  0      0      0   0 0 0  0.1 0   0 ]';   % +p
    x0 + [0 0 0  0      0      0   0 0 0  0   0.1 0 ]';   % +q
    x0 + [0 0 0  0      0      0   0 0 0  0   0   0.1]'}; % +r

cols = {'-g','-r','-b','-m','-c','-w'};

% ODE function handle (uses your provided QuadrotorEOM)
f = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, nu, mu, f_hover);

figs = [1 2 3 4 5 6];   % base six figure IDs

for k = 1:numel(ICs)
    x_ic = ICs{k};
    [t, x] = ode45(f, tspan, x_ic);

    Zc = -sum(f_hover);  u_hist = repmat([Zc;0;0;0],1,numel(t));

    % >>> unique figures per case (makes 36 total)
    figs_case = figs + 6*(k-1);
    PlotAircraftSim(t.', x.', u_hist, figs_case, cols{k});
end

case_labels = { ...
    'Case A: +5° Roll', ...
    'Case B: +5° Pitch', ...
    'Case C: +5° Yaw', ...
    'Case D: +0.1 rad/s p', ...
    'Case E: +0.1 rad/s q', ...
    'Case F: +0.1 rad/s r' };

for k = 1:numel(ICs)
    figs_case = [1 2 3 4 5 6] + 6*(k-1);
    for fid = figs_case
        fh  = figure(fid);
        axs = findall(fh,'Type','axes');
        for ax = axs.'
            if ~isempty(findobj(ax,'Type','line')) || ~isempty(findobj(ax,'Type','scatter'))
                legend(ax, case_labels{k}, 'Location','best');
            end
        end
    end
end
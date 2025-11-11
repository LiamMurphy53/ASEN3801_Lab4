% Part 2.5 — Feedback ONLY (self-contained, noisy on errors)
dbstop if error; warning('on','all'); clc; close all; clear;

% --- Params ---
g=9.81; m=0.068; d=0.060; km=0.0024;
I=diag([5.8e-5,7.2e-5,1.0e-4]); nu=1e-3; mu=2e-6;
tspan=[0 10]; x0=zeros(12,1);

% --- Preflight: required files present? ---
req = {'QuadrotorEOMwithRateFeedback','QuadrotorEOMwithFeedback', ...
       'RotationDerivativeFeedback','ComputeMotorForces','FUNCplotAircraftSim','PlotAircraftSim'};
for i=1:numel(req)
    w = which(req{i});
    fprintf('%-30s : %s\n', req{i}, tern(isempty(w),'NOT FOUND',w));
end
% pick feedback EOM that exists
if exist('QuadrotorEOMwithRateFeedback','file')
    fControlled = @(t,var) QuadrotorEOMwithRateFeedback(t,var,g,m,I,nu,mu);
elseif exist('QuadrotorEOMwithFeedback','file')
    fControlled = @(t,var) QuadrotorEOMwithFeedback(t,var,g,m,I,nu,mu);
else
    error('No feedback EOM file found in path.');
end
% choose plotting function name that exists
if exist('PlotAircraftSim','file')
    plotfun = @PlotAircraftSim;
elseif exist('FUNCplotAircraftSim','file')
    plotfun = @FUNCplotAircraftSim;
else
    error('No plotting function (PlotAircraftSim / FUNCplotAircraftSim) found.');
end

% --- ICs for 2.1(d–f) ---
ICs = { x0+[zeros(9,1);0.1;0;0], x0+[zeros(9,1);0;0.1;0], x0+[zeros(9,1);0;0;0.1] };
labels = {'Case D: +0.1 p','Case E: +0.1 q','Case F: +0.1 r'};
cols = {'-r','-b','-m'};
figs0 = [1 2 3 4 5 6];

% --- Run ---
for k=1:numel(ICs)
    x_ic = ICs{k};
    fprintf('Running %s ...\n', labels{k});
    [t,x] = ode45(fControlled,tspan,x_ic);  % N×12

    % recover commanded forces/moments for plotting
    N = numel(t);
    Fc = zeros(3,N); Gc = zeros(3,N);
    for i=1:N
        [Fc(:,i),Gc(:,i)] = RotationDerivativeFeedback(x(i,:).',m,g);
    end
    u_hist = [Fc(3,:); Gc];                 % [Zc; Lc; Mc; Nc], 4×N

    figs_case = figs0 + 6*(k-1);
    feval(plotfun, t.', x.', u_hist, figs_case, cols{k});

    % one-entry legend per axes
    fh = arrayfun(@figure, figs_case);
    for f=fh.'
        axs = findall(f,'Type','axes');
        for ax=axs.'
            hold(ax,'on'); plot(ax,NaN,NaN,cols{k},'LineWidth',1.4,'DisplayName',labels{k});
            legend(ax,'Location','best'); legend(ax,'boxoff');
        end
    end
end

% --- helper ---
function out=tern(cond,a,b)
if cond, out=a; else, out=b; end
end
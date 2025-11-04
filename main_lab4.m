


const = struct();
const.g = 9.81; %m/s^2
const.m = 0.068; %kg
const.km = 0.0024; %N*m/(N)
const.d = 0.06; %m
const.nu = 1*10^(-3); %N/(m/s)^2
const.mu = 2*10^(-6); %N*m/(rad/s)^2
const.I = [5.8*10^(-5), 0, 0; 0, 7.2*10^(-5), 0; 0, 0, 1*10^(-4)]; %kg*m^2

tspan = [0, 10]; %seconds
            % x y z, u, v, w, phi, theta, psi, p, q, r
%statevector_0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
statevector_0 = [10, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0];

% motor forces calculation at trim
f = (const.m*const.g)/4;
motor_forces = [f, f, f, f];



[t, statevector] = ode45(@(t,statevector) QuadrotorEOM(t,statevector, const, motor_forces),tspan,statevector_0);

function var_dot = QuadrotorEOM(t, var, const, motor_forces)
    %decompose
    x = var(1);
    y = var(2);
    z = var(3);
    u = var(4);
    v = var(5);
    w = var(6);
    phi = var(7);
    theta = var(8);
    psi = var(9);
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
        0, sin(phi)*sec(theta), cos(phi)*sec(theta)] * 0';

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

    
    v_e_dot = [r*v-q*w; p*w-r*u; q*u-p*v] + (1/const.m)*(W + [0; 0; Zc]);
    
    Ix = const.I(1,1);
    Iy = const.I(2,2);
    Iz = const.I(3,3);

    omega = [((Iy-Iz)/Ix)*q*r; ((Iz-Ix)/Iy)*p*r; ((Ix-Iy)/Iz)*p*q] + [1/Ix, 0, 0; 0, 1/Iy, 0; 0, 0, 1/Iz]*([Lc; Mc; Nc]);

    var_dot = [pos_dot(1); pos_dot(2); pos_dot(3); v_e_dot(1); v_e_dot(2); v_e_dot(3); e_dot(1); e_dot(2); e_dot(3); omega(1); omega(2); omega(3)];
end

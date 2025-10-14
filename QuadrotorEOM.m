function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
x_E = var(1); y_E = var(2); z_E = var(3);

phi = var(4); theta = var(5); psi = var(6);

u_E = var(7); v_E = var(8); w_E = var(9);

p = var(10); q = var(11); r = var(12);

VeB = [u_E; v_E; w_E];
omega = [p; q; r];
Cbe = C_E(phi,theta,psi);


A = [ -1 -1 -1 -1;
    -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2);
    d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2);
    km -km km -km];
Moments = A*motor_forces;
Zc = Moments(1);
Lc = Moments(2);
Mc = Moments(3);
Nc = Moments(4);

FgB = Cbe * [0;0;m*g];

F_z = FgB + [0;0;Zc];
M = Lc + Mc + Nc;

VeB_dot   = (F_z - cross(omega, m*VeB)) / m;
omega_dot = I \ ( M - cross(omega, I*omega) );

rE_dot  = (Cbe.') * VeB;
eul_dot = T(phi,theta,psi) * omega;

var_dot = [rE_dot; eul_dot; VeB_dot; omega_dot];


end 

function T_ang = T(phi, theta,~)
cphi=cos(phi); sphi=sin(phi);
cth=cos(theta); sth=sin(theta);
T_ang = [ 1,  sphi*sth/cth,  cphi*sth/cth;
      0,  cphi,         -sphi;
      0,  sphi/cth,      cphi/cth ];
end
function Cbe = C_E(phi, theta, psi)
cphi=cos(phi); sphi=sin(phi);
cth=cos(theta); sth=sin(theta);
cpsi=cos(psi); spsi=sin(psi);
Cbe = [ cth*cpsi,                     cth*spsi,                  -sth;
      sphi*sth*cpsi-cphi*spsi,     sphi*sth*spsi+cphi*cpsi,   sphi*cth;
      cphi*sth*cpsi+sphi*spsi,     cphi*sth*spsi-sphi*cpsi,   cphi*cth ];
end
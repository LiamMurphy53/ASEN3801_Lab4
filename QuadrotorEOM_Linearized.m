function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)


dx_E   = var(1);  
dy_E   = var(2);  
dz_E   = var(3);  

dphi   = var(4);
dtheta = var(5);
dpsi   = var(6);  

du     = var(7);
dv     = var(8);
dw     = var(9);

dp     = var(10);
dq     = var(11);
dr     = var(12);


dFx = deltaFc(1);
dFy = deltaFc(2);
dFz = deltaFc(3);


var_dot = zeros(12,1);


var_dot(1) = du; 
var_dot(2) = dv;
var_dot(3) = dw;


var_dot(4) = dp; 
var_dot(5) = dq; 
var_dot(6) = dr; 


var_dot(7) = -g * dtheta + dFx/m;
var_dot(8) =  g * dphi   + dFy/m;
var_dot(9) =  dFz/m;

omega_dot  = I \ deltaGc; 

var_dot(10) = omega_dot(1);
var_dot(11) = omega_dot(2); 
var_dot(12) = omega_dot(3); 
end
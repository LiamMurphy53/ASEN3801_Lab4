function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

p = var(10); q = var(11); r = var(12);

Fc = [0; 0; -m*g];

k_rate = 0.004;          
Gc = -k_rate * [p; q; r];
end
function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)

    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);   

    d = 0.06;
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);   

    var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end

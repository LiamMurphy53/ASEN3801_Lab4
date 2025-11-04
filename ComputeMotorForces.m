function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

inertMatrix = [ -1,          -1,          -1,          -1;
      -d/sqrt(2),  -d/sqrt(2),   d/sqrt(2),   d/sqrt(2);
       d/sqrt(2),  -d/sqrt(2),  -d/sqrt(2),   d/sqrt(2);
       km,         -km,          km,         -km ];

x = [Fc(3); Gc(:)];

motor_forces = inertMatrix \ x;

end
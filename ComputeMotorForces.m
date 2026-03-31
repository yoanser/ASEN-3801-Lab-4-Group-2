function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
droot2 = d/(sqrt(2));
X_c = Fc(1); % Likely to stay 0 the majority of the time
Y_c = Fc(2); % ^Ditto
Z_c = Fc(3);

L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);

ControlVec = [-1 -1 -1 -1; -(droot2)  -(droot2) droot2 droot2; droot2 -(droot2) -(droot2) droot2; km -km km -km];


motor_forces = ControlVec \ [Z_c;L_c;M_c;N_c]; %Computes motor forces by using inverse of Control Vector


%motor_forces = [f_1, f_2, f_3, f_4]';

end